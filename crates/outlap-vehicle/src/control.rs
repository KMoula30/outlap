// SPDX-License-Identifier: AGPL-3.0-only
//! The `control`/`actuate` blocks that close the transient loop: the ideal deterministic [`Driver`]
//! (MacAdam-style preview steer + curvature feed-forward + PI speed tracking of the QSS profile), the
//! [`Powertrain`] (static split → per-wheel drive force off the best-gear traction envelope +
//! balance-bar friction braking + the regen blend), and the [`TorqueVectoring`] yaw-moment allocator.
//! HANDOFF §7.7 (driver) and §8.0/§8.2 (splits, allocation); MacAdam 1981 for the preview law — see
//! `docs/theory/driver.md` and `docs/theory/transient_control.md`.
//!
//! All blocks are pure and generic over `f32`/`f64` and allocate nothing in the loop. The driver is
//! a `control`-phase block (Decision #38: built-in core controller, external plugin trait deferred);
//! it also carries the **augmented-ODE** speed integral, so its [`Block::derivatives`] writes both
//! the steer/throttle/brake bus signals *and* the integral derivative `ξ̇ = v_ref − v_x` into the
//! shared derivative buffer — the RK sweep advances `ξ` alongside the chassis DOF (the PI loop is a
//! continuous state, not a step-boundary snapshot). The powertrain and the allocator are
//! `actuate`-phase blocks, in that order (the allocator augments the wheel torques the powertrain
//! wrote, inside the friction ellipse the tyre/load blocks set).
//!
//! **The engaged gear indexes no force here (PR6).** The wheel-force ceiling stays the *best-gear*
//! [`Powertrain::traction`] envelope, as the QSS tier already picks gears instantaneously. The shift
//! FSM (`outlap_transient::control::Shifter`) therefore acts on this block through exactly one
//! channel — the [`ActuationChannels::torque_scale`] torque interruption — and a stochastic shift
//! delay rides on its step-boundary event queue. A "wander off the line" driver error belongs on the
//! preview channels below, so neither future Monte-Carlo error is blocked by this actuation.

use num_traits::Float;

use outlap_core::block::{Block, Phase, Ports};
use outlap_core::bus::{Bus, CoreSignal, WheelSignal, WHEELS};
use outlap_core::interp::MonotoneCubic;
use outlap_core::state::{ChassisState, ControllerState, DerivView, StateView};

use crate::params::{ActuationChannels, RoadChannels};

/// Minimum driver look-ahead distance, m — floors `L_p = v_x·t_preview` so the preview terms stay
/// well-posed at low speed (and the feed-forward accel denominator never hits zero).
pub const PREVIEW_FLOOR_M: f64 = 2.0;

/// The driver look-ahead distance `L_p = max(v_x·t_preview, PREVIEW_FLOOR_M)`, m. Shared by the
/// [`Driver`] block and the orchestrator that samples the preview line channels, so both evaluate the
/// preview at the *same* station for a given stage state (no drift between sampler and consumer).
#[inline]
#[must_use]
pub fn preview_distance<T: Float>(vx: T, preview_time: T) -> T {
    let floor = T::from(PREVIEW_FLOOR_M).unwrap_or_else(T::one);
    (vx.max(T::zero()) * preview_time).max(floor)
}

/// The ideal deterministic driver (Decision #21): two loops on the QSS target line, plus a minimal
/// yaw-rate stabiliser (the front-steer half of what PR6's torque vectoring will do actively) so the
/// car can lap a real circuit without spinning. See `docs/theory/driver.md`.
///
/// **Slide factor** — `β = atan2(v_y, v_x)` is the sideslip; `recover = clamp(k_slip·(|β| − β_lim),
/// 0, 1)` grows 0 → 1 as the rear steps out and gates both loops toward recovery.
///
/// **Steering** — curvature feed-forward + preview path law + yaw-rate stabilisation:
/// ```text
/// δ_ff   = κ_ref(s+L_p) · (L + K_us · v_x²)                  (understeer-gradient feed-forward)
/// r_tgt  = v_x · κ_ref(s+L_p)                                (reference yaw rate)
/// n_pred = n + L_p · sin ψ_rel                               (offset predicted at the preview point)
/// δ_fb   = (1−recover)·k_prev·(n_ref(s+L_p) − n_pred) − k_ψ·ψ_rel + k_r·(1+5·recover)·(r_tgt − r) − k_β·β
/// δ      = clamp(δ_ff + δ_fb, ±δ_max)
/// ```
/// Damping the yaw to `r_tgt` (not 0) makes the driver **counter-steer** when the car over-rotates;
/// as `recover → 1` the path term (which would steer further into the corner) fades and the
/// counter-steer gain escalates. When gripping (`recover ≈ 0`) it is a gentle path law that does not
/// touch clean cornering. `K_us` is the vehicle's own understeer gradient
/// (`T1Vehicle::understeer_gradient`, Decision #8).
///
/// **Speed** — PI tracking of the QSS profile with a preview feed-forward, power cut while sliding:
/// ```text
/// e_v      = v_ref(s) − v_x
/// a_ff     = (v_ref(s+L_p) − v_x) · v_x / L_p                (accel to reach the previewed speed)
/// u        = a_ff / a_scale + k_p·e_v + k_i·ξ                (a_scale = gg-headroom usable accel)
/// throttle = max(clamp(u, ±1), 0) · (1 − recover),  brake = max(−clamp(u, ±1), 0)
/// ξ̇      = e_v   (0 when the pedal is saturated — by ±1 or the slide cut — anti-windup)
/// ```
/// Scaling the throttle by `(1 − recover)` removes the power that is overloading a sliding rear. `ξ`
/// is the augmented-ODE integral state ([`ControllerState::SpeedIntegral`]); its derivative is written
/// here and the RK sweep integrates it. The point-mass QSS profile spends the whole grip envelope
/// longitudinally; a transient rear-drive car needs a grip margin on a real track until PR6's active
/// yaw moment lands.
#[derive(Clone, Copy, Debug)]
pub struct Driver<T> {
    /// Wheelbase `L`, m (curvature feed-forward arm).
    pub wheelbase: T,
    /// Understeer gradient `K_us`, rad·s²/m (`> 0` understeer) — the curvature FF speed term.
    pub understeer_gradient: T,
    /// MacAdam preview time `t_preview`, s (look-ahead `L_p = v_x·t_preview`).
    pub preview_time: T,
    /// Preview-error steer gain `k_prev`, rad/m.
    pub preview_gain: T,
    /// Heading-error gain `k_ψ`, rad/rad.
    pub heading_gain: T,
    /// Yaw-rate damping gain `k_r`, rad/(rad/s).
    pub yaw_damping: T,
    /// Steer saturation `δ_max`, rad.
    pub max_steer: T,
    /// Speed-loop proportional gain `k_p`, pedal per (m/s).
    pub speed_kp: T,
    /// Speed-loop integral gain `k_i`, pedal per (m/s·s) = pedal/m.
    pub speed_ki: T,
    /// Feed-forward normalising accel `a_scale`, m/s² (the gg-headroom usable acceleration): the
    /// demanded longitudinal accel `a_ff` is mapped onto the `[−1, 1]` pedal axis as `a_ff / a_scale`.
    pub ff_accel_scale: T,
    /// Sideslip magnitude `|β|` (rad) at which slide recovery begins (the linear-grip limit): past it,
    /// path-following + throttle fade and the counter-steer escalates.
    pub slip_limit: T,
    /// Slide-recovery ramp rate per rad of sideslip past [`slip_limit`](Self::slip_limit) (1/rad); the
    /// `recover` factor reaches 1 (full opposite-lock, no power) at `β = slip_limit + 1/slip_gain`.
    pub slip_gain: T,
    /// Sideslip-damping steer gain `k_β`, rad/rad: `δ −= k_β·β` re-aligns the heading with the
    /// velocity vector. The yaw-rate damper only sees *rotational* slides (`r` far from `r_target`);
    /// a **translational** slide — the car crabbing off the line with `r ≈ r_target ≈ 0` after a
    /// corner-exit — is invisible to it, and the path term is faded exactly when it happens. This
    /// term is the correction that closes that gap.
    pub sideslip_damping: T,
    /// Drive-wheel slip ratio at which the pedal governor starts cutting (near the force peak).
    pub traction_slip_limit: T,
    /// Governor cut rate per unit of slip past the limit (pedal fraction per slip ratio).
    pub traction_slip_gain: T,
    /// Anti-windup clamp on `|ξ|`, m (a backstop; conditional integration is the primary limiter).
    pub integral_limit: T,
    /// Interned road channels (reads the current + preview target-line channels).
    pub road: RoadChannels,
}

impl<T: Float> Block<T> for Driver<T> {
    fn phase(&self) -> Phase {
        Phase::Control
    }

    fn ports(&self) -> Ports {
        Ports::new(
            vec![
                self.road.v_ref.index(),
                self.road.n_ref_preview.index(),
                self.road.kappa_ref_preview.index(),
                self.road.v_ref_preview.index(),
            ],
            vec![
                CoreSignal::Steer as usize,
                CoreSignal::Throttle as usize,
                CoreSignal::Brake as usize,
            ],
        )
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, dx: &mut DerivView<T>, lane: usize) {
        let (zero, one) = (T::zero(), T::one());
        let n = x.chassis(ChassisState::N);
        let psi = x.chassis(ChassisState::PsiRel);
        let vx = x.chassis(ChassisState::Vx);
        let vy = x.chassis(ChassisState::Vy);
        let r = x.chassis(ChassisState::YawRate);
        let xi = x.controller(ControllerState::SpeedIntegral);
        let vx_pos = vx.max(zero);

        let v_ref = bus.get_channel(self.road.v_ref, lane);
        let n_prev = bus.get_channel(self.road.n_ref_preview, lane);
        let kappa_prev = bus.get_channel(self.road.kappa_ref_preview, lane);
        let v_prev = bus.get_channel(self.road.v_ref_preview, lane);

        // Same look-ahead the orchestrator sampled the preview channels at.
        let lp = preview_distance(vx, self.preview_time);

        // --- Slide state: how far the sideslip is past its linear-grip value (0 = gripping, → 1 as
        //     the car slides). It gates BOTH loops: as the rear steps out, path-following and throttle
        //     give way to slide recovery (the minimal yaw stabilisation the ideal driver needs to lap
        //     a real track before PR6's torque-vectoring controller lands). ---
        let beta = vy.atan2(vx_pos.max(T::from(0.5).unwrap_or_else(T::one)));
        let slide = (beta.abs() - self.slip_limit).max(zero);
        let recover = (self.slip_gain * slide).min(one); // 0 gripping, 1 fully sliding
        let grip = one - recover; // authority left for path-following / throttle

        // --- Steering: curvature feed-forward + preview path law + yaw-rate stabilisation. ---
        let d_ff = kappa_prev * (self.wheelbase + self.understeer_gradient * vx_pos * vx_pos);
        // Predict the offset at the preview point from the body heading (a well-damped path law).
        let (spsi, _cpsi) = psi.sin_cos();
        let n_pred = n + lp * spsi;
        let e_lat = n_prev - n_pred;
        // Reference yaw rate for the previewed corner. Damping the yaw to `r_target` (not to 0) makes
        // the driver **counter-steer** when the car over-rotates (|r| > |r_target| ⇒ oversteer); the
        // path term — which would steer further *into* the corner and worsen the slide — is faded out
        // by `grip`, so recovery wins once the rear is loose.
        let r_target = vx_pos * kappa_prev;
        // The yaw-tracking (counter-steer) gain escalates sharply with the slide — gentle damping when
        // gripping, strong opposite lock once the rear is loose — so the recovery has the authority to
        // catch a slide without over-damping normal cornering (where `recover ≈ 0`).
        let yaw_gain = self.yaw_damping * (one + T::from(5.0).unwrap_or_else(T::one) * recover);
        let d_fb = grip * self.preview_gain * e_lat - self.heading_gain * psi
            + yaw_gain * (r_target - r)
            - self.sideslip_damping * beta;
        let steer = (d_ff + d_fb).max(-self.max_steer).min(self.max_steer);
        bus.set(CoreSignal::Steer, lane, steer);

        // --- Speed: preview feed-forward + PI, cut back as the car slides (grip). ---
        let e_v = v_ref - vx;
        let a_ff = (v_prev - vx_pos) * vx_pos / lp;
        let u = a_ff / self.ff_accel_scale + self.speed_kp * e_v + self.speed_ki * xi;
        let u_sat = u.max(-one).min(one);
        let want_thr = u_sat.max(zero);
        let want_brk = (-u_sat).max(zero);
        // Wheel-slip governor: an ideal driver modulates the pedal against drive wheelspin (with a
        // race gearing, low-gear torque is a multiple of the grip limit, so an unmodulated pedal
        // fraction lights up the driven axle mid-exit — the measured slide trigger). Proportional
        // cut on the worst positive lagged slip ratio; braking slips are negative and untouched.
        let mut kappa_max = T::zero();
        for w in 0..outlap_core::bus::WHEELS {
            kappa_max = kappa_max.max(x.relax(outlap_core::state::RelaxState::Kappa, w));
        }
        let over = (kappa_max - self.traction_slip_limit).max(zero);
        let tc = (one - self.traction_slip_gain * over).max(zero);
        let throttle = want_thr * grip * tc; // no power while sliding or spinning up
        let brake = want_brk;
        bus.set(CoreSignal::Throttle, lane, throttle);
        bus.set(CoreSignal::Brake, lane, brake);

        // --- Augmented-ODE speed integral: ξ̇ = e_v, halted when the pedal is saturated (by the
        //     ±1 clamp OR the grip limit) and the error would wind it further in — conditional
        //     integration anti-windup. The RK sweep integrates ξ. ---
        let sat_hi = (throttle < want_thr) || (u >= one);
        let sat_lo = (brake < want_brk) || (u <= -one);
        let dxi = if (sat_hi && e_v > zero) || (sat_lo && e_v < zero) {
            zero
        } else {
            e_v
        };
        dx.set_controller(ControllerState::SpeedIntegral, dxi);
    }
}

/// Speed below which regen fades linearly to zero, m/s. Real controllers hand braking back to the
/// calipers at a walking pace: torque control degrades, the recoverable energy is negligible, and the
/// machine must release the wheel before it stops. Also keeps `P = F·v` well-behaved at `v → 0`.
pub const REGEN_FADE_SPEED_MPS: f64 = 2.0;

/// One axle's regen machine (`ptm/1.2` envelope sampled at assembly). A machine can only ever brake
/// the wheels it drives, so the front and rear machines are independent actuators sharing one pack.
#[derive(Clone, Debug)]
pub struct AxleRegen<T> {
    /// Peak regen **braking wheel force** this axle's machine can produce vs vehicle speed, N (≥ 0)
    /// — `outlap_qss::T1Vehicle::max_regen_force_by_axle` sampled into the shared monotone cubic.
    pub force_max: MonotoneCubic<T>,
    /// Mechanical→electrical recovery efficiency (machine + inverter + driveline), `0..1`. A
    /// documented constant proxy: the mapped `.ptm` efficiency drives QSS energy accounting, and the
    /// wasm-clean block must never touch a `.ptm` table.
    pub efficiency: T,
    /// Blend authority: the largest fraction of *this axle's* commanded brake torque the machine is
    /// allowed to take (`brakes.regen_blend.max_regen_frac`). `1` ⇒ take everything the envelope and
    /// the pack allow; the calipers always supply whatever is left.
    pub authority: T,
}

/// Regen-brake blend parameters (`brakes.regen_blend`, HANDOFF §7.6): **series (blended) braking**, as
/// production EVs and full hybrids do it.
///
/// Under braking each axle's machine absorbs as much of *its own axle's* commanded brake torque as it
/// can, and the friction brakes supply the deficit. Three ceilings bound the machine, exactly as a
/// real BMS/inverter pair enforces them:
///
/// 1. **Available regen torque** — the machine's speed-dependent braking envelope ([`AxleRegen`]).
/// 2. **Battery charge acceptance** — the pack's ceiling at the current charge *and temperature* (a cold
///    pack cannot take a fast charge), published on the slow clock into
///    [`regen_limit_w`](ActuationChannels::regen_limit_w). Shared by both axles.
/// 3. **Blend authority** — a policy cap on the machine's share of the axle.
///
/// The machine substitutes for the calipers *inside* the commanded brake torque rather than adding to
/// it, so the axle total, the wheel deceleration, and hence the whole trajectory are identical with
/// regen on or off (Decision #11); only the recovered energy differs. Whatever the machine cannot
/// take, the calipers take.
#[derive(Clone, Debug)]
pub struct RegenParams<T> {
    /// Whether regen blending is active (a battery and at least one driven electric machine).
    pub enabled: bool,
    /// The front-axle machine, if the front wheels are driven by one.
    pub front: Option<AxleRegen<T>>,
    /// The rear-axle machine, if the rear wheels are driven by one.
    pub rear: Option<AxleRegen<T>>,
}

impl<T: Float> RegenParams<T> {
    /// A disabled blend: no machine, no recovery, calipers do all the braking.
    #[must_use]
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            front: None,
            rear: None,
        }
    }

    /// The low-speed fade factor at speed `vx`, `0..1` (see [`REGEN_FADE_SPEED_MPS`]).
    fn fade(vx: T) -> T {
        let fade_speed = T::from(REGEN_FADE_SPEED_MPS).unwrap_or_else(T::one);
        if fade_speed <= T::zero() {
            T::one()
        } else {
            (vx / fade_speed).min(T::one()).max(T::zero())
        }
    }
}

/// Drive/brake actuation with the PR6 rule-based control layer: throttle scales the **best-gear**
/// wheel-force ceiling [`traction`](Self::traction) (the QSS instantaneous-shift envelope) — modulated
/// by the shift FSM's [`torque_scale`](ActuationChannels::torque_scale) so a gear change costs the
/// §8.2 torque interruption — distributed by the static [`drive_weight`](Self::drive_weight); brake
/// scales a constant maximum brake torque split by the balance bar; and, under braking, each driven
/// axle's machine recovers energy inside that commanded torque per [`regen`](Self::regen). The
/// yaw-moment torque-vectoring allocation is a *separate* `actuate` block ([`TorqueVectoring`]) so the
/// two concerns compose.
///
/// The static split reproduces the differential's behaviour at *equal* per-wheel grip (open ⇒ 50/50;
/// locked/LSD ⇒ 50/50 with no load transfer to bias against); the grip-proportional, friction-ellipse
/// diff allocation needs per-wheel `F_z`, which is only available after load transfer, and lands with
/// the post-v1 QP allocator.
#[derive(Clone, Debug)]
pub struct Powertrain<T> {
    /// Best-gear maximum wheel drive force vs vehicle speed `F_drive_max(v)`, N — the QSS traction
    /// ceiling sampled into the shared monotone-cubic interpolant.
    pub traction: MonotoneCubic<T>,
    /// Per-wheel drive-force weights `[FL, FR, RL, RR]` (static axle/side split over the driven
    /// wheels; sums to 1, zero on undriven corners).
    pub drive_weight: [T; WHEELS],
    /// Effective rolling radius per wheel, m (wheel drive force → applied wheel torque).
    pub radius: [T; WHEELS],
    /// Maximum total brake torque, N·m.
    pub max_brake_torque: T,
    /// Front brake-force bias, `0..1`.
    pub brake_front_bias: T,
    /// Per-axle regen-brake blend parameters (series braking).
    pub regen: RegenParams<T>,
    /// Interned actuation channels (reads shift `torque_scale` + `regen_limit_w`; publishes
    /// `regen_power_w` and the per-axle machine braking torques).
    pub actuation: ActuationChannels,
}

impl<T: Float> Block<T> for Powertrain<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let ch = |sig: WheelSignal, w: usize| base + (sig as usize) * WHEELS + w;
        let mut writes = vec![
            self.actuation.regen_power_w.index(),
            self.actuation.traction_power_w.index(),
            self.actuation.regen_torque_front_nm.index(),
            self.actuation.regen_torque_rear_nm.index(),
        ];
        for w in 0..WHEELS {
            writes.push(ch(WheelSignal::WheelDriveTorque, w));
            writes.push(ch(WheelSignal::WheelBrakeTorque, w));
        }
        Ports::new(
            vec![
                CoreSignal::Throttle as usize,
                CoreSignal::Brake as usize,
                self.actuation.torque_scale.index(),
                self.actuation.regen_limit_w.index(),
            ],
            writes,
        )
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, _dx: &mut DerivView<T>, lane: usize) {
        let (zero, one) = (T::zero(), T::one());
        let vx = x.chassis(ChassisState::Vx).max(zero);
        let throttle = bus.get(CoreSignal::Throttle, lane);
        let brake = bus.get(CoreSignal::Brake, lane);
        // Shift-FSM torque scale (0 during the cut, ramping to 1 on re-engagement); the orchestrator
        // publishes it every step, so a bare `1.0` never leaks in from a cleared bus (assembly guard).
        let torque_scale = bus.get_channel(self.actuation.torque_scale, lane);

        // Total available wheel drive force at this speed (best gear), gated by throttle and the
        // shift torque interruption.
        let f_avail = throttle * torque_scale * self.traction.eval(vx);

        let total_brake = brake * self.max_brake_torque;
        let two = one + one;
        let front_share = self.brake_front_bias * total_brake / two; // per front wheel
        let rear_share = (one - self.brake_front_bias) * total_brake / two; // per rear wheel

        // Commanded braking *force* per axle (what the calipers would deliver alone), N.
        let mut axle_brake_force = [zero; 2];
        for w in 0..WHEELS {
            // Wheel drive torque = (share of available drive force) × rolling radius, so at steady
            // spin the tyre delivers ≈ that force (grip permitting; wheelspin caps it via the tyre).
            let drive = f_avail * self.drive_weight[w] * self.radius[w];
            bus.set_wheel(WheelSignal::WheelDriveTorque, w, lane, drive);
            let brake_w = if w < 2 { front_share } else { rear_share };
            // The axle *total* brake torque — friction plus regen. This is what the tyre responds to,
            // so the regen blend below never changes it and the trajectory is regen-invariant (#11).
            bus.set_wheel(WheelSignal::WheelBrakeTorque, w, lane, brake_w);
            let axle = usize::from(w >= 2);
            if self.radius[w] > zero {
                axle_brake_force[axle] = axle_brake_force[axle] + brake_w / self.radius[w];
            }
        }

        let (regen_power, axle_regen_torque) = self.blend_regen(vx, axle_brake_force, bus, lane);
        bus.set_channel(self.actuation.regen_power_w, lane, regen_power);
        bus.set_channel(
            self.actuation.traction_power_w,
            lane,
            self.traction_draw(vx, f_avail),
        );
        bus.set_channel(
            self.actuation.regen_torque_front_nm,
            lane,
            axle_regen_torque[0],
        );
        bus.set_channel(
            self.actuation.regen_torque_rear_nm,
            lane,
            axle_regen_torque[1],
        );
    }
}

impl<T: Float> Powertrain<T> {
    /// The mean rolling radius of axle `a` (`0` front, `1` rear), m.
    fn axle_radius(&self, a: usize) -> T {
        let two = T::one() + T::one();
        (self.radius[2 * a] + self.radius[2 * a + 1]) / two
    }

    /// The electrical **traction** power drawn from the pack this step, W (≥ 0) — the mechanical drive
    /// power an electric machine puts down (`F_drive,axle · v_x`), divided by its motoring efficiency.
    ///
    /// Only axles carrying a machine ([`RegenParams`] `front`/`rear`) draw from the pack; an undriven
    /// or engine-driven axle draws nothing (an ICE burns fuel, not charge). For a pure EV this is the
    /// whole drive power; for a hybrid the engine's share is not split out at this tier, so the pack
    /// draw is an upper bound (a documented T2 simplification — the `.ptm` split lands with the QP
    /// powertrain). Netted against [`Self::blend_regen`]'s recovery by the slow stack, so the pack
    /// state of charge falls under power and rises under braking, as a real stint does.
    fn traction_draw(&self, vx: T, f_avail: T) -> T {
        let zero = T::zero();
        if !self.regen.enabled {
            return zero;
        }
        let machines = [self.regen.front.as_ref(), self.regen.rear.as_ref()];
        let mut elec = zero;
        for (a, machine) in machines.iter().enumerate() {
            let Some(m) = machine else { continue };
            if m.efficiency <= zero {
                continue;
            }
            let axle_weight = self.drive_weight[2 * a] + self.drive_weight[2 * a + 1];
            let mech = (f_avail * axle_weight * vx).max(zero); // mechanical drive power on this axle
            elec = elec + mech / m.efficiency;
        }
        elec
    }

    /// **Series (blended) braking.** Each axle's machine takes as much of *its own* commanded braking
    /// force as its envelope and blend authority allow; the calipers supply the deficit. The two
    /// machines then share the pack's single charge-acceptance ceiling: if their combined electrical
    /// demand exceeds it, both are scaled back proportionally and the calipers pick up the slack.
    ///
    /// Returns `(electrical regen power W, [front, rear] regen brake torque N·m)`. The wheel brake
    /// torques on the bus are the axle *totals* and are deliberately untouched, so the car decelerates
    /// identically whether the energy went into the pack or into the discs (Decision #11).
    fn blend_regen(
        &self,
        vx: T,
        axle_brake_force: [T; 2],
        bus: &Bus<T>,
        lane: usize,
    ) -> (T, [T; 2]) {
        let zero = T::zero();
        if !self.regen.enabled {
            return (zero, [zero; 2]);
        }
        let fade = RegenParams::fade(vx);
        let machines = [self.regen.front.as_ref(), self.regen.rear.as_ref()];

        // (1) Per-axle mechanical capture: bounded by the machine's speed-dependent braking envelope,
        //     by the blend authority, and — implicitly — by what the driver actually asked for.
        let mut mech = [zero; 2]; // mechanical power taken by each machine, W
        let mut force = [zero; 2]; // regen braking force at each axle, N
        for a in 0..2 {
            let Some(m) = machines[a] else { continue };
            let demand = (m.authority.max(zero).min(T::one())) * axle_brake_force[a].max(zero);
            let capability = m.force_max.eval(vx).max(zero) * fade;
            force[a] = demand.min(capability);
            mech[a] = force[a] * vx;
        }

        // (2) The pack is shared. Scale both machines back together when their combined electrical
        //     demand exceeds the charge-acceptance ceiling (SoC *and* temperature dependent), so the
        //     split between axles is preserved and the calipers absorb the remainder.
        let limit = bus
            .get_channel(self.actuation.regen_limit_w, lane)
            .max(zero);
        let mut elec = [zero; 2];
        for a in 0..2 {
            if let Some(m) = machines[a] {
                elec[a] = mech[a] * m.efficiency;
            }
        }
        let demand_w = elec[0] + elec[1];
        if demand_w > limit {
            let scale = if demand_w > zero {
                limit / demand_w
            } else {
                zero
            };
            for a in 0..2 {
                elec[a] = elec[a] * scale;
                force[a] = force[a] * scale;
            }
        }

        // (3) Report the machine's braking torque per axle (telemetry); the calipers supply
        //     `axle_brake_force − force` without any further bookkeeping — the bus already carries the
        //     total, and the difference is the friction share.
        let torque = [
            force[0] * self.axle_radius(0),
            force[1] * self.axle_radius(1),
        ];
        ((elec[0] + elec[1]).max(zero), torque)
    }
}

/// The result of a torque-vectoring allocation: the per-wheel longitudinal force deltas and the yaw
/// moment they actually realise (≤ the demand once the friction-ellipse limits bind).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct YawAllocation<T> {
    /// Per-wheel longitudinal wheel-frame force delta `Δf_x` `[FL, FR, RL, RR]`, N (`+` adds drive,
    /// `−` adds brake). Each keeps the wheel inside its friction ellipse.
    pub delta_fx: [T; WHEELS],
    /// The yaw moment `ΔM_z` the deltas realise (N·m, +CCW) — `sign(demand)·min(|demand|, feasible)`.
    pub moment_nm: T,
}

/// Allocate a demanded yaw moment `demand_nm` (+CCW) across the wheels as a set of longitudinal
/// force deltas, each clamped inside that wheel's **friction ellipse** (HANDOFF §8.0). This is the
/// rule-based v1 allocator; its interface (a feasibility set + a proportional fill) is shaped so a QP
/// allocator can replace the body without touching callers (Decision #2).
///
/// For each wheel the longitudinal grip headroom is `f_x,max = √((μ·F_z)² − F_y²)` (0 when the wheel
/// is already at its lateral limit). Each wheel pushes its delta in the sign that adds toward the
/// demanded moment (`−sign(demand·y_i)`), by an amount that never takes `|F_x + Δf_x|` past
/// `f_x,max`; drive-incapable wheels may only *brake* (negative delta). The demand is filled
/// proportionally to each wheel's contributable moment `|y_i|·headroom`, so the realised moment is
/// `min(|demand|, Σ|y_i|·headroom)` — a genuine per-wheel split, never a lumped moment injection.
///
/// `mu` is the friction-ellipse radius coefficient (the vehicle's representative peak grip); `y` are
/// the lateral arms (+left); `drive_capable[i]` marks wheels a machine can add drive torque to.
#[must_use]
pub fn allocate_yaw_moment<T: Float>(
    demand_nm: T,
    y: &[T; WHEELS],
    fx: &[T; WHEELS],
    fy: &[T; WHEELS],
    fz: &[T; WHEELS],
    mu: T,
    drive_capable: &[bool; WHEELS],
) -> YawAllocation<T> {
    let zero = T::zero();
    let mut sign_dir = [zero; WHEELS];
    let mut head = [zero; WHEELS];
    let mut max_moment = zero;
    for i in 0..WHEELS {
        let radius_sq = (mu * fz[i]) * (mu * fz[i]) - fy[i] * fy[i];
        let fx_max = if radius_sq > zero {
            radius_sq.sqrt()
        } else {
            zero
        };
        // The delta sign that adds toward the demand at this wheel: contribution to Mz is −y_i·Δf_x,
        // so a positive demand wants Δf_x with sign −sign(y_i) (·sign(demand) for either polarity).
        let s = if demand_nm * y[i] > zero {
            -T::one()
        } else if demand_nm * y[i] < zero {
            T::one()
        } else {
            zero
        };
        // Grip headroom in that direction (drive-incapable wheels cannot add positive/drive delta).
        let h = if s > zero {
            if drive_capable[i] {
                (fx_max - fx[i]).max(zero)
            } else {
                zero
            }
        } else if s < zero {
            (fx_max + fx[i]).max(zero)
        } else {
            zero
        };
        sign_dir[i] = s;
        head[i] = h;
        max_moment = max_moment + y[i].abs() * h;
    }
    // Fill the demand proportionally to each wheel's contributable moment; clamp to the feasible set.
    let scale = if max_moment > zero {
        (demand_nm.abs() / max_moment).min(T::one())
    } else {
        zero
    };
    let mut delta_fx = [zero; WHEELS];
    for i in 0..WHEELS {
        delta_fx[i] = sign_dir[i] * scale * head[i];
    }
    let moment_nm = demand_nm.signum() * scale * max_moment;
    YawAllocation {
        delta_fx,
        moment_nm,
    }
}

/// The rule-based **torque-vectoring** allocator block (HANDOFF §8.0; Decision #2): it drives the yaw
/// rate toward the reference `r_target = v_x·κ_ref` with `ΔM_z = k_yaw·(r_target − r)`, then realises
/// that moment **physically** by adding per-wheel drive/brake torque within the friction ellipse
/// ([`allocate_yaw_moment`]) — so the moment emerges through the tyres, not as a lumped external
/// couple. An `actuate`-phase block that runs *after* the powertrain (whose wheel torques it augments)
/// and the tyre/load blocks (whose forces/loads set the ellipse). Disabled ⇒ a no-op that only zeroes
/// its telemetry channel, so a car that does not enable TV is byte-identical to the pre-PR6 lap.
#[derive(Clone, Copy, Debug)]
pub struct TorqueVectoring<T> {
    /// Whether torque vectoring is active.
    pub enabled: bool,
    /// Yaw-rate feedback gain `k_yaw`, N·m per rad/s.
    pub k_yaw: T,
    /// Hard cap on `|ΔM_z|`, N·m (`+∞` when the schema leaves `max_yaw_moment_nm` unset).
    pub max_moment: T,
    /// Friction-ellipse radius coefficient `μ` (the vehicle's representative peak grip).
    pub mu: T,
    /// Per-wheel lateral arm `y_i` (+left), m.
    pub y: [T; WHEELS],
    /// Per-wheel effective rolling radius, m (force↔torque at the contact patch).
    pub radius: [T; WHEELS],
    /// Which wheels a machine can add *drive* torque to (all wheels may brake).
    pub drive_capable: [bool; WHEELS],
    /// Interned road channels (reads `κ_ref` for the reference yaw rate).
    pub road: RoadChannels,
    /// Interned actuation channels (publishes the realised `yaw_moment_cmd`).
    pub actuation: ActuationChannels,
}

impl<T: Float> Block<T> for TorqueVectoring<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let ch = |sig: WheelSignal, w: usize| base + (sig as usize) * WHEELS + w;
        let mut reads = vec![self.road.kappa_ref.index()];
        let mut writes = vec![self.actuation.yaw_moment_cmd.index()];
        for sig in [
            WheelSignal::TireFx,
            WheelSignal::TireFy,
            WheelSignal::TireFz,
        ] {
            for w in 0..WHEELS {
                reads.push(ch(sig, w));
            }
        }
        for w in 0..WHEELS {
            // Read-modify-write: augment the powertrain's per-wheel drive/brake torques.
            reads.push(ch(WheelSignal::WheelDriveTorque, w));
            reads.push(ch(WheelSignal::WheelBrakeTorque, w));
            writes.push(ch(WheelSignal::WheelDriveTorque, w));
            writes.push(ch(WheelSignal::WheelBrakeTorque, w));
        }
        Ports::new(reads, writes)
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, _dx: &mut DerivView<T>, lane: usize) {
        let zero = T::zero();
        if !self.enabled {
            bus.set_channel(self.actuation.yaw_moment_cmd, lane, zero);
            return;
        }
        let vx = x.chassis(ChassisState::Vx).max(zero);
        let r = x.chassis(ChassisState::YawRate);
        let kappa_ref = bus.get_channel(self.road.kappa_ref, lane);
        let r_target = vx * kappa_ref;
        let demand = (self.k_yaw * (r_target - r))
            .max(-self.max_moment)
            .min(self.max_moment);

        let mut fx = [zero; WHEELS];
        let mut fy = [zero; WHEELS];
        let mut fz = [zero; WHEELS];
        for i in 0..WHEELS {
            fx[i] = bus.get_wheel(WheelSignal::TireFx, i, lane);
            fy[i] = bus.get_wheel(WheelSignal::TireFy, i, lane);
            fz[i] = bus.get_wheel(WheelSignal::TireFz, i, lane);
        }
        let alloc =
            allocate_yaw_moment(demand, &self.y, &fx, &fy, &fz, self.mu, &self.drive_capable);

        // Realise the deltas as extra per-wheel drive/brake torque (force · rolling radius): the wheel
        // spin responds, the slip evolves, and the tyre produces the extra longitudinal force — the
        // yaw moment emerges through the tyres over the relaxation lag, not as a lumped couple.
        for i in 0..WHEELS {
            let dtq = alloc.delta_fx[i] * self.radius[i];
            if dtq >= zero {
                let cur = bus.get_wheel(WheelSignal::WheelDriveTorque, i, lane);
                bus.set_wheel(WheelSignal::WheelDriveTorque, i, lane, cur + dtq);
            } else {
                let cur = bus.get_wheel(WheelSignal::WheelBrakeTorque, i, lane);
                bus.set_wheel(WheelSignal::WheelBrakeTorque, i, lane, cur - dtq);
            }
        }
        bus.set_channel(self.actuation.yaw_moment_cmd, lane, alloc.moment_nm);
    }
}

/// Resolve the static per-wheel drive-force weights `[FL, FR, RL, RR]` from the driven-wheel mask and
/// the [`DriveControl::Split`](outlap_schema::vehicle::Split) shares (axle `front`, side `left`),
/// summing to 1 over the driven wheels (zero elsewhere). Absent shares default to the driven mass
/// distribution (front share from the driven axles) and an even side split. Used at assembly (never
/// in the loop).
///
/// # Panics
/// Panics if no wheel is driven (an assembly-time topology error the loader rejects earlier).
#[must_use]
pub fn drive_weights<T: Float>(
    driven: [bool; WHEELS],
    front_share: Option<T>,
    left_share: Option<T>,
) -> [T; WHEELS] {
    let zero = T::zero();
    let half = T::from(0.5).unwrap_or_else(T::one);
    let front_driven = driven[0] || driven[1];
    let rear_driven = driven[2] || driven[3];
    // Axle shares: honour an explicit `front` split, else split by which axles are driven.
    let (fa, ra) = match front_share {
        Some(f) => (f, T::one() - f),
        None => match (front_driven, rear_driven) {
            (true, true) => (half, half),
            (true, false) => (T::one(), zero),
            (false, true) => (zero, T::one()),
            (false, false) => (zero, zero),
        },
    };
    let ls = left_share.unwrap_or(half);
    let rs = T::one() - ls;
    let mut w = [
        fa * ls, // FL
        fa * rs, // FR
        ra * ls, // RL
        ra * rs, // RR
    ];
    // Mask undriven corners, then renormalise so the driven weights sum to 1.
    let mut sum = zero;
    for i in 0..WHEELS {
        if !driven[i] {
            w[i] = zero;
        }
        sum = sum + w[i];
    }
    assert!(sum > zero, "no driven wheel to allocate drive torque to");
    for wi in &mut w {
        *wi = *wi / sum;
    }
    w
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    use outlap_core::bus::ChannelInterner;
    use outlap_core::state::fast_slot_count;

    use crate::params::{ActuationChannels, RoadChannels};

    /// Symmetric rear-drive test geometry: track ±0.8 m, driven rear.
    const Y: [f64; WHEELS] = [0.8, -0.8, 0.8, -0.8];
    const REAR_DRIVEN: [bool; WHEELS] = [false, false, true, true];

    #[test]
    fn allocation_stays_inside_the_friction_ellipse() {
        // Sweep demand, load, and the current force state; every commanded delta must keep each
        // wheel inside its ellipse |Fx + ΔFx| ≤ √((μFz)² − Fy²), and |ΔMz| ≤ |demand|.
        let mu = 1.5;
        for &demand in &[-4000.0, -500.0, 0.0, 500.0, 4000.0, 40_000.0] {
            for &fz in &[1500.0, 4000.0, 8000.0] {
                for &fy in &[0.0, 2000.0, 5000.0] {
                    for &fx0 in &[-3000.0, 0.0, 3000.0] {
                        let fzs = [fz; WHEELS];
                        let fys = [fy, -fy, fy, -fy];
                        let fxs = [fx0; WHEELS];
                        let a = allocate_yaw_moment(demand, &Y, &fxs, &fys, &fzs, mu, &REAR_DRIVEN);
                        for i in 0..WHEELS {
                            let fx_max_sq = (mu * fz) * (mu * fz) - fy * fy;
                            let fx_max = if fx_max_sq > 0.0 {
                                fx_max_sq.sqrt()
                            } else {
                                0.0
                            };
                            let total = fxs[i] + a.delta_fx[i];
                            // Containment holds whenever the baseline was itself feasible.
                            if fx0.abs() <= fx_max + 1e-6 {
                                assert!(
                                    total.abs() <= fx_max + 1e-6,
                                    "wheel {i} left the ellipse: |{total}| > {fx_max} \
                                     (demand={demand}, fz={fz}, fy={fy}, fx0={fx0})"
                                );
                            }
                            // Drive-incapable (front) wheels never gain drive torque (positive Δ).
                            if !REAR_DRIVEN[i] {
                                assert!(a.delta_fx[i] <= 1e-9, "front wheel {i} gained drive");
                            }
                        }
                        assert!(
                            a.moment_nm.abs() <= demand.abs() + 1e-6,
                            "realised moment {} exceeds demand {demand}",
                            a.moment_nm
                        );
                        if demand != 0.0 && a.moment_nm != 0.0 {
                            assert_eq!(
                                a.moment_nm.signum(),
                                demand.signum(),
                                "realised moment fights the demand"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn allocation_realises_the_full_demand_when_grip_is_ample() {
        // Ample grip (light lateral use, high load): the demand is met in full as a couple.
        let mu = 1.6;
        let fz = [6000.0; WHEELS];
        let fy = [500.0, -500.0, 500.0, -500.0];
        let fx = [0.0; WHEELS];
        let a = allocate_yaw_moment(1200.0, &Y, &fx, &fy, &fz, mu, &REAR_DRIVEN);
        assert!(
            (a.moment_nm - 1200.0).abs() < 1e-6,
            "moment={}",
            a.moment_nm
        );
        // Rear couple: outer (right, y<0) drives +, inner (left, y>0) brakes −.
        assert!(a.delta_fx[3] > 0.0 && a.delta_fx[2] < 0.0);
    }

    #[test]
    fn disabled_torque_vectoring_is_a_no_op() {
        let mut it = ChannelInterner::new();
        let road = RoadChannels::intern(&mut it);
        let actuation = ActuationChannels::intern(&mut it);
        let tv = TorqueVectoring {
            enabled: false,
            k_yaw: 500.0,
            max_moment: f64::INFINITY,
            mu: 1.5,
            y: Y,
            radius: [0.33; WHEELS],
            drive_capable: REAR_DRIVEN,
            road,
            actuation,
        };
        let mut bus = Bus::<f64>::with_interner(&it, 1);
        // Seed non-trivial forces + a yaw error, then run: the wheel torques must be untouched.
        for w in 0..WHEELS {
            bus.set_wheel(WheelSignal::TireFz, w, 0, 5000.0);
            bus.set_wheel(WheelSignal::WheelDriveTorque, w, 0, 100.0);
        }
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::Vx as usize] = 40.0;
        fast[ChassisState::YawRate as usize] = 0.5;
        let sv = StateView::new(&fast, 1, 0);
        let mut dfast = vec![0.0; fast_slot_count()];
        let mut dv = DerivView::new(&mut dfast, 1, 0);
        tv.derivatives(&sv, &mut bus, &mut dv, 0);
        for w in 0..WHEELS {
            assert_eq!(bus.get_wheel(WheelSignal::WheelDriveTorque, w, 0), 100.0);
        }
        assert_eq!(bus.get_channel(actuation.yaw_moment_cmd, 0), 0.0);
    }

    #[test]
    fn regen_recovers_only_within_the_battery_ceiling_without_changing_brake_torque() {
        let mut it = ChannelInterner::new();
        let actuation = ActuationChannels::intern(&mut it);
        let pt = Powertrain {
            traction: MonotoneCubic::new(vec![0.0, 100.0], vec![5000.0, 5000.0]).unwrap(),
            drive_weight: [0.0, 0.0, 0.5, 0.5],
            radius: [0.33; WHEELS],
            max_brake_torque: 6000.0,
            brake_front_bias: 0.6,
            // A rear machine with an effectively unbounded braking envelope and full blend authority,
            // so the *battery* ceiling is the only thing that can bind in this test.
            regen: RegenParams {
                enabled: true,
                front: None,
                rear: Some(AxleRegen {
                    force_max: MonotoneCubic::new(vec![0.0, 100.0], vec![1.0e9, 1.0e9]).unwrap(),
                    efficiency: 0.9,
                    authority: 1.0,
                }),
            },
            actuation,
        };
        let mut bus = Bus::<f64>::with_interner(&it, 1);
        bus.set_channel(actuation.torque_scale, 0, 1.0);
        bus.set(CoreSignal::Brake, 0, 1.0);
        let mut fast = vec![0.0; fast_slot_count()];
        fast[ChassisState::Vx as usize] = 50.0;
        let sv = StateView::new(&fast, 1, 0);
        let mut dfast = vec![0.0; fast_slot_count()];

        // Case 1: a generous ceiling — regen equals mech·η of the driven-wheel braking power.
        bus.set_channel(actuation.regen_limit_w, 0, 1.0e9);
        {
            let mut dv = DerivView::new(&mut dfast, 1, 0);
            pt.derivatives(&sv, &mut bus, &mut dv, 0);
        }
        let brake_rear = bus.get_wheel(WheelSignal::WheelBrakeTorque, 2, 0);
        assert!(brake_rear > 0.0, "rear friction brake torque still applied");
        let mech = 2.0 * (brake_rear / 0.33) * 50.0; // both rear wheels
        let recovered = bus.get_channel(actuation.regen_power_w, 0);
        assert!(
            (recovered - mech * 0.9).abs() < 1e-6,
            "recovered={recovered}, mech={mech}"
        );

        // Case 2: a tight ceiling clips the recovery, and the friction brake torque is unchanged.
        bus.set_channel(actuation.regen_limit_w, 0, 5000.0);
        {
            let mut dv = DerivView::new(&mut dfast, 1, 0);
            pt.derivatives(&sv, &mut bus, &mut dv, 0);
        }
        assert_eq!(
            bus.get_wheel(WheelSignal::WheelBrakeTorque, 2, 0),
            brake_rear,
            "regen substitutes for the calipers inside the axle total; it never alters it"
        );
        assert!((bus.get_channel(actuation.regen_power_w, 0) - 5000.0).abs() < 1e-6);
    }
}

#[cfg(test)]
mod regen_tests {
    //! Series (blended) braking: the machine takes what it can of its **own** axle's commanded brake
    //! torque, and the calipers supply the deficit. These tests drive `blend_regen` directly, so each
    //! ceiling — machine envelope, blend authority, pack charge acceptance, low-speed fade — is
    //! isolated one at a time.
    #![allow(clippy::float_cmp)] // the no-op / full-handoff cases assert exact zeros and exact shares.

    use super::{AxleRegen, Powertrain, RegenParams, REGEN_FADE_SPEED_MPS};
    use crate::params::ActuationChannels;
    use outlap_core::bus::{Bus, ChannelInterner};
    use outlap_core::interp::MonotoneCubic;

    const R: f64 = 0.33; // rolling radius, m
    const ETA: f64 = 0.9; // machine + inverter recovery

    /// A machine whose braking envelope is a flat `force_max` newtons at every speed.
    fn machine(force_max: f64, authority: f64) -> AxleRegen<f64> {
        AxleRegen {
            force_max: MonotoneCubic::new(vec![0.0, 200.0], vec![force_max, force_max]).unwrap(),
            efficiency: ETA,
            authority,
        }
    }

    /// A powertrain with the given per-axle machines, plus a bus carrying `pack_limit_w` as the
    /// battery's charge-acceptance ceiling.
    fn rig(
        front: Option<AxleRegen<f64>>,
        rear: Option<AxleRegen<f64>>,
        pack_limit_w: f64,
    ) -> (Powertrain<f64>, Bus<f64>) {
        let mut it = ChannelInterner::new();
        let actuation = ActuationChannels::intern(&mut it);
        let mut bus = Bus::with_interner(&it, 1);
        bus.set_channel(actuation.regen_limit_w, 0, pack_limit_w);
        let pt = Powertrain {
            traction: MonotoneCubic::new(vec![0.0, 200.0], vec![1.0, 1.0]).unwrap(),
            drive_weight: [0.0, 0.0, 0.5, 0.5],
            radius: [R; 4],
            max_brake_torque: 6000.0,
            brake_front_bias: 0.6,
            regen: RegenParams {
                enabled: front.is_some() || rear.is_some(),
                front,
                rear,
            },
            actuation,
        };
        (pt, bus)
    }

    /// Well above the fade-out speed, so the fade factor is exactly 1.
    const V: f64 = 30.0;

    /// The machine takes its whole authorised share and the calipers take the rest. Here the machine
    /// is oversized and the pack is thirsty, so it takes the axle's *entire* commanded braking force —
    /// the calipers contribute nothing on that axle.
    #[test]
    fn machine_takes_its_share_and_the_calipers_take_the_rest() {
        let (pt, bus) = rig(None, Some(machine(1.0e6, 1.0)), 1.0e12);
        let cmd = [4000.0, 3000.0]; // commanded braking force per axle, N
        let (power, torque) = pt.blend_regen(V, cmd, &bus, 0);

        // Front axle has no machine: the calipers do all of it.
        assert_eq!(torque[0], 0.0, "no front machine ⇒ no front regen");
        // Rear machine absorbs the whole rear command; the rear calipers supply 0.
        assert_eq!(
            torque[1],
            cmd[1] * R,
            "rear machine took the whole rear axle"
        );
        assert_eq!(power, cmd[1] * V * ETA, "electrical yield = F·v·η");
    }

    /// **A machine may only brake its own axle.** An oversized rear machine never reaches across to
    /// absorb the front axle's braking, however much headroom it has.
    #[test]
    fn a_machine_never_brakes_the_other_axle() {
        let (pt, bus) = rig(None, Some(machine(1.0e6, 1.0)), 1.0e12);
        let cmd = [9000.0, 100.0]; // huge front demand, tiny rear demand
        let (power, torque) = pt.blend_regen(V, cmd, &bus, 0);
        assert_eq!(torque[0], 0.0);
        assert_eq!(
            torque[1],
            cmd[1] * R,
            "the rear machine is bounded by the *rear* command, not the front's"
        );
        assert_eq!(power, cmd[1] * V * ETA);
    }

    /// **A cold (or full) pack hands the braking back to the calipers.** With zero charge acceptance
    /// the machine takes nothing: regen power and regen torque are exactly zero, and — because the bus
    /// still carries the axle's full commanded brake torque — the car decelerates exactly the same.
    #[test]
    fn a_pack_that_cannot_accept_charge_hands_braking_to_the_calipers() {
        let (pt, bus) = rig(Some(machine(1.0e6, 1.0)), Some(machine(1.0e6, 1.0)), 0.0);
        let (power, torque) = pt.blend_regen(V, [4000.0, 3000.0], &bus, 0);
        assert_eq!(power, 0.0, "a pack at 0 W acceptance recovers nothing");
        assert_eq!(
            torque,
            [0.0, 0.0],
            "…and the machines apply no braking torque"
        );
    }

    /// **Available regen torque caps the machine.** A small machine takes only what its envelope
    /// allows; the calipers cover the shortfall.
    #[test]
    fn the_machine_envelope_caps_the_regen_share() {
        let cap = 800.0; // N of braking force the machine can produce
        let (pt, bus) = rig(None, Some(machine(cap, 1.0)), 1.0e12);
        let cmd = [0.0, 5000.0];
        let (power, torque) = pt.blend_regen(V, cmd, &bus, 0);
        assert_eq!(torque[1], cap * R, "capped by the machine envelope");
        assert_eq!(power, cap * V * ETA);
        // The calipers supply the deficit, and the axle total is untouched.
        let caliper_force = cmd[1] - cap;
        assert!(caliper_force > 0.0, "the calipers are doing the rest");
    }

    /// The blend authority is a policy cap on the machine's share of its axle.
    #[test]
    fn blend_authority_caps_the_machine_share() {
        let (pt, bus) = rig(None, Some(machine(1.0e6, 0.3)), 1.0e12);
        let cmd = [0.0, 5000.0];
        let (_, torque) = pt.blend_regen(V, cmd, &bus, 0);
        assert_eq!(
            torque[1],
            0.3 * cmd[1] * R,
            "30 % authority ⇒ 30 % of the axle"
        );
    }

    /// **One pack, two machines.** When their combined electrical demand exceeds the pack's
    /// acceptance, both are scaled back by the same factor, so the front/rear split is preserved and
    /// the calipers absorb the remainder on each axle.
    #[test]
    fn a_shared_pack_ceiling_scales_both_axles_proportionally() {
        let (pt, bus_open) = rig(
            Some(machine(1.0e6, 1.0)),
            Some(machine(1.0e6, 1.0)),
            1.0e12, // uncapped
        );
        let cmd = [4000.0, 2000.0];
        let (uncapped_power, uncapped_torque) = pt.blend_regen(V, cmd, &bus_open, 0);

        // Now allow only half of what they wanted.
        let (pt, bus_capped) = rig(
            Some(machine(1.0e6, 1.0)),
            Some(machine(1.0e6, 1.0)),
            uncapped_power / 2.0,
        );
        let (power, torque) = pt.blend_regen(V, cmd, &bus_capped, 0);

        assert!(
            (power - uncapped_power / 2.0).abs() < 1e-9 * uncapped_power,
            "the pack ceiling binds: {power} vs {}",
            uncapped_power / 2.0
        );
        for a in 0..2 {
            assert!(
                (torque[a] - uncapped_torque[a] / 2.0).abs() < 1e-9 * uncapped_torque[a],
                "axle {a} scaled proportionally: {} vs {}",
                torque[a],
                uncapped_torque[a] / 2.0
            );
        }
        // The 2:1 front/rear split survives the scaling.
        assert!((torque[0] / torque[1] - 2.0).abs() < 1e-9);
    }

    /// Regen fades to nothing at a walking pace: torque control degrades and the machine must release
    /// the wheel before the car stops. At exactly half the fade speed, half the capability remains.
    #[test]
    fn regen_fades_out_at_low_speed() {
        let (pt, bus) = rig(None, Some(machine(1.0e6, 1.0)), 1.0e12);
        let cmd = [0.0, 3000.0];

        let (power_stopped, torque_stopped) = pt.blend_regen(0.0, cmd, &bus, 0);
        assert_eq!(power_stopped, 0.0, "a stopped car recovers nothing");
        assert_eq!(torque_stopped[1], 0.0);

        // At half the fade speed the *envelope* is halved; the command still exceeds it here, so the
        // machine is envelope-bound and the fade is directly visible.
        let v_half = REGEN_FADE_SPEED_MPS / 2.0;
        let small = machine(cmd[1], 1.0); // envelope exactly equals the command
        let (pt_small, bus2) = rig(None, Some(small), 1.0e12);
        let (_, torque_half) = pt_small.blend_regen(v_half, cmd, &bus2, 0);
        assert!(
            (torque_half[1] - 0.5 * cmd[1] * R).abs() < 1e-9 * cmd[1] * R,
            "half fade speed ⇒ half the braking authority: {}",
            torque_half[1]
        );
        let _ = pt;
    }

    /// A disabled blend is a byte-exact no-op — the calipers do everything, as on a car with no
    /// machine at all.
    #[test]
    fn disabled_regen_is_a_no_op() {
        let (mut pt, bus) = rig(Some(machine(1.0e6, 1.0)), Some(machine(1.0e6, 1.0)), 1.0e12);
        pt.regen.enabled = false;
        let (power, torque) = pt.blend_regen(V, [4000.0, 3000.0], &bus, 0);
        assert_eq!(power, 0.0);
        assert_eq!(torque, [0.0, 0.0]);
    }

    /// The machine can never recover more mechanical power than the driver asked the brakes for on
    /// that axle — regen substitutes for the calipers, it never adds deceleration.
    #[test]
    fn regen_never_exceeds_the_commanded_braking() {
        for &authority in &[0.0, 0.25, 0.6, 1.0] {
            for &cap in &[10.0, 500.0, 1.0e6] {
                let (pt, bus) = rig(
                    Some(machine(cap, authority)),
                    Some(machine(cap, authority)),
                    1.0e12,
                );
                let cmd = [4000.0, 3000.0];
                let (_, torque) = pt.blend_regen(V, cmd, &bus, 0);
                for a in 0..2 {
                    assert!(
                        torque[a] <= cmd[a] * R + 1e-9,
                        "axle {a}: machine torque {} exceeds the command {}",
                        torque[a],
                        cmd[a] * R
                    );
                }
            }
        }
    }
}
