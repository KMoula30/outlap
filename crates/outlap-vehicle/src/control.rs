// SPDX-License-Identifier: AGPL-3.0-only
//! The `control`/`actuate` blocks that close the transient loop: the ideal deterministic [`Driver`]
//! (MacAdam-style preview steer + curvature feed-forward + PI speed tracking of the QSS profile) and
//! the minimal-actuation [`Powertrain`] (static split → per-wheel drive force off the best-gear
//! traction envelope + balance-bar friction braking). HANDOFF §7.7 (driver) and §8.0/§8.2 (splits);
//! MacAdam 1981 for the preview law — see `docs/theory/driver.md`.
//!
//! Both blocks are pure and generic over `f32`/`f64` and allocate nothing in the loop. The driver is
//! a `control`-phase block (Decision #38: built-in core controller, external plugin trait deferred);
//! it also carries the **augmented-ODE** speed integral, so its [`Block::derivatives`] writes both
//! the steer/throttle/brake bus signals *and* the integral derivative `ξ̇ = v_ref − v_x` into the
//! shared derivative buffer — the RK sweep advances `ξ` alongside the chassis DOF (the PI loop is a
//! continuous state, not a step-boundary snapshot). The powertrain is an `actuate`-phase block.
//!
//! **Scope (PR5, "first closed-loop laps"):** the shift is *instantaneous & ideal* — the best-gear
//! wheel-force ceiling is baked into the [`Powertrain::traction`] envelope (as the QSS tier already
//! picks gears instantaneously), so there is no gear index or shift event yet. The full torque-cut →
//! ratio-swap → clutch-ramp shift FSM (with `shift_time_s`, on the step-boundary event queue) and
//! the yaw-moment torque-vectoring allocator are **PR6**; a stochastic shift delay belongs on that
//! event queue, and a "wander off the line" driver error belongs on the preview channels below — so
//! neither future Monte-Carlo error is blocked by this minimal actuation.

use num_traits::Float;

use outlap_core::block::{Block, Phase, Ports};
use outlap_core::bus::{Bus, CoreSignal, WheelSignal, WHEELS};
use outlap_core::interp::MonotoneCubic;
use outlap_core::state::{ChassisState, ControllerState, DerivView, StateView};

use crate::params::RoadChannels;

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
/// δ_fb   = (1−recover)·k_prev·(n_ref(s+L_p) − n_pred) − k_ψ·ψ_rel + k_r·(1+5·recover)·(r_tgt − r)
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
        let d_fb =
            grip * self.preview_gain * e_lat - self.heading_gain * psi + yaw_gain * (r_target - r);
        let steer = (d_ff + d_fb).max(-self.max_steer).min(self.max_steer);
        bus.set(CoreSignal::Steer, lane, steer);

        // --- Speed: preview feed-forward + PI, cut back as the car slides (grip). ---
        let e_v = v_ref - vx;
        let a_ff = (v_prev - vx_pos) * vx_pos / lp;
        let u = a_ff / self.ff_accel_scale + self.speed_kp * e_v + self.speed_ki * xi;
        let u_sat = u.max(-one).min(one);
        let want_thr = u_sat.max(zero);
        let want_brk = (-u_sat).max(zero);
        let throttle = want_thr * grip; // no power while the rear is sliding
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

/// Minimal drive/brake actuation (**superseded by PR6's shift FSM + torque-vectoring allocator**):
/// throttle scales the **best-gear** wheel-force ceiling [`traction`](Self::traction) (the QSS
/// instantaneous-shift envelope), distributed to the wheels by the pre-resolved static
/// [`drive_weight`](Self::drive_weight) (axle/side split × driven mask); brake scales a constant
/// maximum brake torque split by the balance bar. No gear index, no torque vectoring.
///
/// The static split reproduces the differential's behaviour at *equal* per-wheel grip (open ⇒ 50/50;
/// locked/LSD ⇒ 50/50 with no load transfer to bias against); the grip-proportional, friction-ellipse
/// diff allocation needs per-wheel `F_z`, which is only available after load transfer, and lands with
/// the PR6 allocator.
#[derive(Clone, Debug)]
pub struct Powertrain<T> {
    /// Best-gear maximum wheel drive force vs vehicle speed `F_drive_max(v)`, N — the QSS traction
    /// ceiling sampled into the shared monotone-cubic interpolant (instantaneous ideal shift).
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
}

impl<T: Float> Block<T> for Powertrain<T> {
    fn phase(&self) -> Phase {
        Phase::Actuate
    }

    fn ports(&self) -> Ports {
        let base = CoreSignal::COUNT as usize;
        let ch = |sig: WheelSignal, w: usize| base + (sig as usize) * WHEELS + w;
        let mut writes = Vec::new();
        for w in 0..WHEELS {
            writes.push(ch(WheelSignal::WheelDriveTorque, w));
            writes.push(ch(WheelSignal::WheelBrakeTorque, w));
        }
        Ports::new(
            vec![CoreSignal::Throttle as usize, CoreSignal::Brake as usize],
            writes,
        )
    }

    fn derivatives(&self, x: &StateView<T>, bus: &mut Bus<T>, _dx: &mut DerivView<T>, lane: usize) {
        let vx = x.chassis(ChassisState::Vx).max(T::zero());
        let throttle = bus.get(CoreSignal::Throttle, lane);
        let brake = bus.get(CoreSignal::Brake, lane);

        // Total available wheel drive force at this speed (best gear), gated by throttle.
        let f_avail = throttle * self.traction.eval(vx);

        let total_brake = brake * self.max_brake_torque;
        let two = T::one() + T::one();
        let front_share = self.brake_front_bias * total_brake / two; // per front wheel
        let rear_share = (T::one() - self.brake_front_bias) * total_brake / two; // per rear wheel

        for w in 0..WHEELS {
            // Wheel drive torque = (share of available drive force) × rolling radius, so at steady
            // spin the tyre delivers ≈ that force (grip permitting; wheelspin caps it via the tyre).
            let drive = f_avail * self.drive_weight[w] * self.radius[w];
            bus.set_wheel(WheelSignal::WheelDriveTorque, w, lane, drive);
            let brake_w = if w < 2 { front_share } else { rear_share };
            bus.set_wheel(WheelSignal::WheelBrakeTorque, w, lane, brake_w);
        }
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
