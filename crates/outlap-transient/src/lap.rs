// SPDX-License-Identifier: AGPL-3.0-only
//! The **lap-orchestration skeleton**: assemble the T2 blocks, run the fixed-step split integrator,
//! and produce the time-indexed [`TransientLap`] (HANDOFF §11.2, PR4).
//!
//! The solver **receives** the QSS artifacts (envelope-derived speed target, racing line, road
//! geometry) as a [`LineTable`] — it never computes or caches them, so the crate stays wasm-clean
//! (Decision #2 scaffolding home; the envelope disk cache stays at the Python boundary).
//!
//! # Split integration (per fixed step `dt`)
//!
//! 1. **Load-transfer coupling** — resolve the algebraic `F_z` accelerations per the recorded
//!    `fz_coupling` (one-step-lag reuses the previous step's `(a_x, a_y)`; fixed-point runs a few
//!    force→accel Picard iterations at the step start).
//! 2. **Relaxation sub-step** — advance the per-wheel lagged slip `(κ, α)` with the exact-exponential
//!    update (`outlap_core::relax`), frozen across the RK stages.
//! 3. **Runge–Kutta sweep** — advance the 10 chassis DOF with the [`SimArena`] Butcher-generic
//!    explicit RK (Heun default). Each stage re-evaluates the block chain
//!    (`sense → control → actuate → integrate`) at the stage state; the chassis is the sole
//!    derivative writer.
//!
//! The block execution order is fixed (the T2 block set is fixed; a general enum/plugin dispatch is
//! deferred with the plugin trait, Decision #38) and **asserted to equal the assembler-produced
//! [`Schedule`]** in the crate tests, so the topological ordering is honoured and determinism holds.

use num_traits::Float;

use outlap_core::assembler::{assemble, BlockSpec, Schedule};
use outlap_core::block::Block;
use outlap_core::bus::{Bus, ChannelInterner, CoreSignal, WheelSignal, WHEELS};
use outlap_core::integrator::{RkMethod, SimArena};
use outlap_core::relax::SlowClock;
use outlap_core::state::{
    fast_slot_count, ChassisState, ControllerState, DerivView, RelaxState, StateLayout, StateView,
};
use outlap_schema::sim::FzCoupling;

use outlap_vehicle::{
    preview_distance, relax_wheel, ActuationChannels, Aero, Chassis, Driver, LoadTransfer,
    Powertrain, RoadChannels, Tire, TorqueVectoring,
};

use crate::control::{Shifter, SlowStack};
use crate::line_table::LineTable;
use crate::result::TransientLap;

/// Numerics for a transient run (the resolved subset the stepper needs; recorded in provenance).
#[derive(Clone, Copy, Debug)]
pub struct SimConfig<T> {
    /// Fixed step size, s.
    pub dt: T,
    /// Runge–Kutta method (Heun default; RK4 for convergence studies).
    pub method: RkMethod,
    /// Resolved normal-load coupling mode (Decision #29; T2 auto-default is `FixedPoint`).
    pub fz_coupling: FzCoupling,
    /// Fixed-point (Picard) iterations per step when `fz_coupling == FixedPoint`.
    pub fixed_point_iters: u32,
    /// Slow-clock decimation (advance slow states every N fast steps).
    pub slow_decimation: u32,
    /// Lateral-offset edge clamp `|n| ≤ n_max`, m (guards the curvilinear frame; PR3 spec).
    pub n_max: T,
    /// Arc-length station the lap is seeded at, m (default 0). A cold transient — zero relaxation,
    /// zero yaw, running straight — seeded *at* a corner is unphysical (a real lap arrives moving),
    /// so callers start the lap at a straight; the closed line wraps `s` back through the start.
    pub start_s: T,
}

impl<T: Float> Default for SimConfig<T> {
    fn default() -> Self {
        Self {
            dt: T::from(0.001).expect("1 ms representable"),
            method: RkMethod::Heun,
            fz_coupling: FzCoupling::FixedPoint,
            fixed_point_iters: 3,
            slow_decimation: 20,
            n_max: T::from(20.0).expect("20 representable"),
            start_s: T::zero(),
        }
    }
}

/// Resolved provenance recorded with every transient lap (Decision #13).
#[derive(Clone, Copy, Debug)]
pub struct Provenance {
    /// Resolved step size, s.
    pub dt_s: f64,
    /// Resolved integrator order (Heun: 2, RK4: 4).
    pub integrator_order: u32,
    /// Resolved normal-load coupling.
    pub fz_coupling: FzCoupling,
}

/// The blocks the solver owns, handed over pre-built by the assembly pipeline.
pub struct T2Blocks<T> {
    /// The chassis RHS block.
    pub chassis: Chassis<T>,
    /// The tyre block.
    pub tire: Tire<T>,
    /// The aero block.
    pub aero: Aero<T>,
    /// The load-transfer (algebraic `F_z`) block.
    pub load: LoadTransfer<T>,
    /// The ideal-driver block.
    pub driver: Driver<T>,
    /// The powertrain (drive/brake actuation + regen blend) block.
    pub powertrain: Powertrain<T>,
    /// The torque-vectoring allocator block (a no-op when disabled).
    pub tv: TorqueVectoring<T>,
    /// The interned road channels.
    pub road: RoadChannels,
    /// The interned actuation channels (shift/regen plumbing).
    pub actuation: ActuationChannels,
}

impl<T: Float> T2Blocks<T> {
    /// The assembler-facing port specs, in fixed registration order (used for the [`Schedule`] and
    /// the ordering-determinism test). The torque-vectoring allocator registers after the tyre/load
    /// blocks whose forces/loads it reads and the powertrain whose torques it augments, so the
    /// topological sort places it last in the `actuate` phase (before the `integrate` chassis).
    fn specs(&self) -> [BlockSpec; 7] {
        [
            BlockSpec::new(Block::phase(&self.driver), Block::ports(&self.driver)),
            BlockSpec::new(
                Block::phase(&self.powertrain),
                Block::ports(&self.powertrain),
            ),
            BlockSpec::new(Block::phase(&self.load), Block::ports(&self.load)),
            BlockSpec::new(Block::phase(&self.aero), Block::ports(&self.aero)),
            BlockSpec::new(Block::phase(&self.tire), Block::ports(&self.tire)),
            BlockSpec::new(Block::phase(&self.tv), Block::ports(&self.tv)),
            BlockSpec::new(Block::phase(&self.chassis), Block::ports(&self.chassis)),
        ]
    }
}

/// The T2 lap solver: fixed block set + split integrator over one lane (batch = 1; the batch
/// dimension is reserved for the future GPU tier, HANDOFF §11.3).
pub struct TransientSolver<T> {
    blocks: T2Blocks<T>,
    line: LineTable<T>,
    arena: SimArena<T>,
    cfg: SimConfig<T>,
    // SoA scratch (batch = 1).
    fast: Vec<T>,
    dfast: Vec<T>,
    // Gather buffer + the fast-state slots the RK sweep integrates (chassis DOF + controller states).
    x_int: Vec<T>,
    integrated: Vec<usize>,
    bus: Bus<T>,
    schedule: Schedule,
    slow_clock: SlowClock,
    // Rule-based control layer (PR6): the shift FSM (discrete gear state on the event queue) and the
    // slow-state stack (battery SoC on the decimated clock) are optional artifacts handed in by the
    // caller; absent ⇒ an ideal single-gear, no-regen lap (byte-identical to the PR5 skeleton).
    shifter: Option<Shifter<T>>,
    slow: Option<Box<dyn SlowStack>>,
    actuation: ActuationChannels,
    /// Drive-torque scale published this step (`1` when engaged / no shift FSM).
    torque_scale: T,
    /// Battery regen power ceiling published this step, W (0 without a slow stack).
    regen_limit_w: T,
    /// Regen electrical energy accumulated since the last slow-clock fire, J.
    regen_energy_accum: T,
    /// Fast steps elapsed since the last slow-clock fire (to flush the final partial window at lap
    /// end, so no recovered energy is dropped between the last fire and the finish line).
    slow_pending_steps: u32,
    ax_prev: T,
    ay_prev: T,
    t: T,
}

impl<T: Float> TransientSolver<T> {
    /// Build a solver from the assembled blocks, the line table, the interner (bus width), and the
    /// numerics config, seeding the initial state from the target line at `s = 0`.
    #[must_use]
    pub fn new(
        blocks: T2Blocks<T>,
        line: LineTable<T>,
        interner: &ChannelInterner,
        cfg: SimConfig<T>,
    ) -> Self {
        // The RK sweep integrates the T2 chassis DOF plus the continuous controller states (the
        // driver speed integral); the tyre-relaxation states are advanced separately on the
        // exact-exponential channel and are not in this set.
        let mut integrated = vec![0usize; ChassisState::T2_DOF + ControllerState::COUNT as usize];
        let n_int = StateLayout::t2_integrated_slots(&mut integrated);
        integrated.truncate(n_int);
        let arena = SimArena::for_method(cfg.method, n_int);
        let bus = Bus::with_interner(interner, 1);
        let schedule = assemble(&blocks.specs()).expect("acyclic T2 block set");
        let slow_clock = SlowClock::new(cfg.slow_decimation.max(1));
        let actuation = blocks.actuation;
        let mut solver = Self {
            blocks,
            line,
            arena,
            cfg,
            fast: vec![T::zero(); fast_slot_count()],
            dfast: vec![T::zero(); fast_slot_count()],
            x_int: vec![T::zero(); n_int],
            integrated,
            bus,
            schedule,
            slow_clock,
            shifter: None,
            slow: None,
            actuation,
            torque_scale: T::one(),
            regen_limit_w: T::zero(),
            regen_energy_accum: T::zero(),
            slow_pending_steps: 0,
            ax_prev: T::zero(),
            ay_prev: T::zero(),
            t: T::zero(),
        };
        solver.seed_initial_state();
        solver
    }

    /// Attach a gear-shift FSM (consuming). Without one the lap runs a single fixed gear (the QSS
    /// instantaneous-shift traction ceiling), so a shift never interrupts drive torque.
    #[must_use]
    pub fn with_shifter(mut self, shifter: Shifter<T>) -> Self {
        self.shifter = Some(shifter);
        self
    }

    /// Attach a slow-state stack (consuming) — the battery pack whose charge the decimated slow clock
    /// advances, closing the regen recharge path. Without one, regen is recorded as 0 (no battery).
    #[must_use]
    pub fn with_slow_stack(mut self, slow: Box<dyn SlowStack>) -> Self {
        self.regen_limit_w = T::from(slow.regen_power_limit_w()).unwrap_or_else(T::zero);
        self.slow = Some(slow);
        self
    }

    /// The assembler-produced schedule (registration index order of the block specs).
    #[must_use]
    pub fn schedule(&self) -> &Schedule {
        &self.schedule
    }

    /// Resolved provenance for the run.
    #[must_use]
    pub fn provenance(&self) -> Provenance {
        Provenance {
            dt_s: self.cfg.dt.to_f64().unwrap_or(0.0),
            integrator_order: self.cfg.method.order(),
            fz_coupling: self.cfg.fz_coupling,
        }
    }

    /// The interned road channels (for callers writing tests directly against the bus).
    #[must_use]
    pub fn road_channels(&self) -> RoadChannels {
        self.blocks.road
    }

    /// Seed `[s, n, ψ_rel, v_x, v_y, r, ω₁..₄]` from the target line at `s = start_s`. The yaw rate is
    /// seeded to the corner-consistent `r = v·κ_ref` (zero on a straight) so the cold start does not
    /// shock the car with an instantaneous corner from a straight-ahead initial condition.
    fn seed_initial_state(&mut self) {
        let zero = T::zero();
        let s0 = self.cfg.start_s;
        let v0 = self.line.v_ref(s0).max(zero);
        self.fast[ChassisState::S as usize] = s0;
        self.fast[ChassisState::N as usize] = self.line.n_ref(s0);
        self.fast[ChassisState::PsiRel as usize] = zero;
        self.fast[ChassisState::Vx as usize] = v0;
        self.fast[ChassisState::Vy as usize] = zero;
        self.fast[ChassisState::YawRate as usize] = v0 * self.line.kappa_ref(s0);
        for i in 0..WHEELS {
            let r = self.blocks.chassis.params.wheels.radius[i];
            self.fast[omega_state(i) as usize] = if r > zero { v0 / r } else { zero };
        }
    }

    #[inline]
    fn get_fast(&self, s: ChassisState) -> T {
        self.fast[s as usize]
    }

    /// Clear the bus, publish road + actuation, run the block chain; the chassis writes `dfast`.
    fn eval_rhs(&mut self) {
        eval_rhs_raw(
            &self.blocks,
            &self.line,
            &self.fast,
            &mut self.dfast,
            &mut self.bus,
            self.torque_scale,
            self.regen_limit_w,
        );
    }

    /// Body-frame accelerations `(a_x, a_y)` from the last-evaluated `dfast`.
    fn body_accel(&self) -> (T, T) {
        let vx = self.get_fast(ChassisState::Vx);
        let vy = self.get_fast(ChassisState::Vy);
        let r = self.get_fast(ChassisState::YawRate);
        let vx_dot = self.dfast[ChassisState::Vx as usize];
        let vy_dot = self.dfast[ChassisState::Vy as usize];
        (vx_dot - r * vy, vy_dot + r * vx)
    }

    /// Update the [`LoadTransfer`] operating point from the current state and given accelerations.
    fn set_load_operating_point(&mut self, ax: T, ay: T) {
        let s = self.get_fast(ChassisState::S);
        let vx = self.get_fast(ChassisState::Vx).max(T::zero());
        let g = self.blocks.chassis.params.gravity;
        let (grade, bank) = (self.line.grade(s), self.line.banking(s));
        let g_normal = g * grade.cos() * bank.cos() + self.line.kappa_v(s) * vx * vx;
        self.blocks.load.set_operating_point(vx, g_normal, ax, ay);
    }

    /// Resolve the load-transfer accelerations for this step and advance the relaxation states.
    fn couple_and_relax(&mut self) {
        match self.cfg.fz_coupling {
            FzCoupling::OneStepLag => {
                self.set_load_operating_point(self.ax_prev, self.ay_prev);
                self.eval_rhs();
            }
            FzCoupling::FixedPoint => {
                self.set_load_operating_point(self.ax_prev, self.ay_prev);
                self.eval_rhs();
                for _ in 0..self.cfg.fixed_point_iters {
                    let (ax, ay) = self.body_accel();
                    self.set_load_operating_point(ax, ay);
                    self.eval_rhs();
                }
            }
        }
        let dt = self.cfg.dt;
        let steer = self.bus.get(CoreSignal::Steer, 0);
        let sv = StateView::new(&self.fast, 1, 0);
        let mut new_relax = [(T::zero(), T::zero()); WHEELS];
        for (i, slot) in new_relax.iter_mut().enumerate() {
            let fz = self.bus.get_wheel(WheelSignal::TireFz, i, 0);
            let tg = self.blocks.tire.relax_targets(&sv, i, steer, fz);
            let k0 = sv.relax(RelaxState::Kappa, i);
            let a0 = sv.relax(RelaxState::Alpha, i);
            *slot = relax_wheel(k0, a0, &tg, dt);
        }
        for (i, &(k1, a1)) in new_relax.iter().enumerate() {
            self.fast[StateLayout::relax_slot(RelaxState::Kappa, i)] = k1;
            self.fast[StateLayout::relax_slot(RelaxState::Alpha, i)] = a1;
        }
    }

    /// Advance the simulation by one fixed step (control update + relaxation + RK sweep + slow clock).
    pub fn step(&mut self) {
        let dt = self.cfg.dt;

        // (0) rule-based control layer boundary decision (frozen across the RK sweep, like the
        //     relaxation and load-transfer coupling): advance the shift FSM at the current speed to
        //     get this step's drive-torque scale; the regen ceiling is refreshed on the slow clock.
        if let Some(shifter) = self.shifter.as_mut() {
            let v = self.fast[ChassisState::Vx as usize];
            self.torque_scale = shifter.update(self.t, dt, v);
        }

        // (1) coupling + relaxation (leaves the lagged slip frozen for the RK sweep).
        self.couple_and_relax();

        // (2) RK sweep over the integrated fast states (chassis DOF + the driver speed integral),
        //     re-evaluating the block chain (driver writes the integral derivative, chassis the
        //     chassis-DOF derivatives) at each stage.
        for (k, &slot) in self.integrated.iter().enumerate() {
            self.x_int[k] = self.fast[slot];
        }
        // Disjoint field borrows for the closure (arena/state vs. the block+bus scratch).
        let Self {
            arena,
            x_int,
            integrated,
            fast,
            dfast,
            bus,
            blocks,
            line,
            t,
            torque_scale,
            regen_limit_w,
            ..
        } = self;
        let t_now = *t;
        let (ts, rl) = (*torque_scale, *regen_limit_w);
        arena.step(x_int, t_now, dt, |_ti, xs, dxs| {
            for (k, &slot) in integrated.iter().enumerate() {
                fast[slot] = xs[k];
            }
            eval_rhs_raw(blocks, line, fast, dfast, bus, ts, rl);
            for (k, d) in dxs.iter_mut().enumerate() {
                *d = dfast[integrated[k]];
            }
        });
        for (k, &slot) in self.integrated.iter().enumerate() {
            self.fast[slot] = self.x_int[k];
        }
        // Edge-clamp the lateral offset so the curvilinear frame stays well-posed (PR3 spec).
        let n_slot = ChassisState::N as usize;
        self.fast[n_slot] = self.fast[n_slot].max(-self.cfg.n_max).min(self.cfg.n_max);
        // Backstop clamp on the speed integral (conditional integration is the primary anti-windup).
        let xi_slot = StateLayout::controller_slot(ControllerState::SpeedIntegral);
        let xi_lim = self.blocks.driver.integral_limit;
        self.fast[xi_slot] = self.fast[xi_slot].max(-xi_lim).min(xi_lim);

        // (3) refresh one-step-lag accelerations at the new state. Re-point the load-transfer block
        // at the post-step speed first (using the lagged accel), so the recorded per-wheel F_z and
        // the seed accel are evaluated at the state we just reached, not the pre-step operating point.
        self.set_load_operating_point(self.ax_prev, self.ay_prev);
        self.eval_rhs();
        let (ax, ay) = self.body_accel();
        self.ax_prev = ax;
        self.ay_prev = ay;

        // (4) slow-state clock (Decision #6): accumulate this step's recovered regen energy, and on
        //     the decimated boundary Coulomb-count it into the pack SoC and refresh the regen ceiling
        //     (published on the bus for the powertrain's blend cap on the next steps).
        if self.slow.is_some() {
            let regen_power = self.bus.get_channel(self.actuation.regen_power_w, 0);
            self.regen_energy_accum = self.regen_energy_accum + regen_power * dt;
            self.slow_pending_steps += 1;
        }
        if self.slow_clock.tick() {
            self.advance_slow(dt);
        }

        self.t = self.t + dt;
    }

    /// Advance the slow-state stack by the accumulated regen energy over the pending window, then
    /// refresh the published regen ceiling. The window length is the number of fast steps since the
    /// last advance × `dt` — the full `slow_decimation` on a clock fire, or a shorter partial window
    /// when flushed at lap end — so the energy the powertrain produced reaches the pack exactly.
    fn advance_slow(&mut self, dt: T) {
        let Some(slow) = self.slow.as_mut() else {
            return;
        };
        if self.slow_pending_steps == 0 {
            return;
        }
        let steps = T::from(self.slow_pending_steps).unwrap_or_else(T::one);
        let slow_dt = (dt * steps).to_f64().unwrap_or(0.0);
        let energy = self.regen_energy_accum.to_f64().unwrap_or(0.0);
        let avg_power = if slow_dt > 0.0 { energy / slow_dt } else { 0.0 };
        slow.on_slow_step(slow_dt, avg_power);
        self.regen_energy_accum = T::zero();
        self.slow_pending_steps = 0;
        self.regen_limit_w = T::from(slow.regen_power_limit_w()).unwrap_or_else(T::zero);
    }

    /// Record the current state + bus diagnostics as one row of `lap`.
    fn record(&self, lap: &mut TransientLap<T>) {
        lap.t.push(self.t);
        lap.s.push(self.get_fast(ChassisState::S));
        lap.n.push(self.get_fast(ChassisState::N));
        lap.psi_rel.push(self.get_fast(ChassisState::PsiRel));
        lap.vx.push(self.get_fast(ChassisState::Vx));
        lap.vy.push(self.get_fast(ChassisState::Vy));
        lap.yaw_rate.push(self.get_fast(ChassisState::YawRate));
        let (ax, ay) = self.body_accel();
        lap.ax.push(ax);
        lap.ay.push(ay);
        let mut omega = [T::zero(); WHEELS];
        let mut fz = [T::zero(); WHEELS];
        let mut sk = [T::zero(); WHEELS];
        let mut sa = [T::zero(); WHEELS];
        let mut fx = [T::zero(); WHEELS];
        let mut fy = [T::zero(); WHEELS];
        for i in 0..WHEELS {
            omega[i] = self.get_fast(omega_state(i));
            fz[i] = self.bus.get_wheel(WheelSignal::TireFz, i, 0);
            sk[i] = self.bus.get_wheel(WheelSignal::SlipKappa, i, 0);
            sa[i] = self.bus.get_wheel(WheelSignal::SlipAlpha, i, 0);
            fx[i] = self.bus.get_wheel(WheelSignal::TireFx, i, 0);
            fy[i] = self.bus.get_wheel(WheelSignal::TireFy, i, 0);
        }
        lap.omega.push(omega);
        lap.fz.push(fz);
        lap.slip_kappa.push(sk);
        lap.slip_alpha.push(sa);
        lap.fx.push(fx);
        lap.fy.push(fy);
        lap.steer.push(self.bus.get(CoreSignal::Steer, 0));
        lap.throttle.push(self.bus.get(CoreSignal::Throttle, 0));
        lap.brake.push(self.bus.get(CoreSignal::Brake, 0));
        let pos = self.line.world_position(
            self.get_fast(ChassisState::S),
            self.get_fast(ChassisState::N),
        );
        lap.x.push(pos[0]);
        lap.y.push(pos[1]);
        lap.z.push(pos[2]);
        // Rule-based control-layer telemetry.
        let gear = self.shifter.as_ref().map_or(0, Shifter::gear);
        lap.gear.push(T::from(gear).unwrap_or_else(T::zero));
        lap.torque_scale.push(self.torque_scale);
        lap.yaw_moment_nm
            .push(self.bus.get_channel(self.actuation.yaw_moment_cmd, 0));
        lap.regen_power_w
            .push(self.bus.get_channel(self.actuation.regen_power_w, 0));
        lap.regen_torque_front_nm.push(
            self.bus
                .get_channel(self.actuation.regen_torque_front_nm, 0),
        );
        lap.regen_torque_rear_nm
            .push(self.bus.get_channel(self.actuation.regen_torque_rear_nm, 0));
        if let Some(slow) = self.slow.as_ref() {
            lap.state_of_charge
                .push(T::from(slow.soc()).unwrap_or_else(T::zero));
            lap.pack_temp_c
                .push(T::from(slow.temp_c()).unwrap_or_else(T::zero));
        }
    }

    /// Run until the arc length reaches `s_end` or `max_steps` elapse, recording every step.
    #[must_use]
    pub fn run(&mut self, s_end: T, max_steps: usize) -> TransientLap<T> {
        let mut lap = TransientLap::default();
        // Point the load-transfer block at the seeded state so the initial record's F_z is evaluated
        // at the entry speed (not the constructor default) before the first step runs.
        self.set_load_operating_point(self.ax_prev, self.ay_prev);
        self.eval_rhs(); // populate the bus for the initial record
        self.record(&mut lap);
        for _ in 0..max_steps {
            self.step();
            self.record(&mut lap);
            if self.get_fast(ChassisState::S) >= s_end {
                break;
            }
        }
        // Flush the final partial slow window so recovered regen energy between the last slow-clock
        // fire and the finish line is Coulomb-counted (energy closure at the lap boundary).
        self.advance_slow(self.cfg.dt);
        // Re-stamp the last recorded SoC/temperature with the flushed slow state.
        if let Some(slow) = self.slow.as_ref() {
            if let Some(last) = lap.state_of_charge.last_mut() {
                *last = T::from(slow.soc()).unwrap_or(*last);
            }
            if let Some(last) = lap.pack_temp_c.last_mut() {
                *last = T::from(slow.temp_c()).unwrap_or(*last);
            }
        }
        lap.lap_time_s = self.t;
        lap
    }

    /// Access the full fast-state buffer (tests/diagnostics).
    #[must_use]
    pub fn fast_state(&self) -> &[T] {
        &self.fast
    }
}

/// Publish the road-geometry + target-line channels for the `sense` phase (the solver owns the line
/// table): the current-station geometry at `s`, and the **preview** target-line channels sampled at
/// the driver look-ahead `s + L_p(v_x)` that the MacAdam preview steer and speed feed-forward read.
fn publish_road<T: Float>(
    line: &LineTable<T>,
    road: &RoadChannels,
    bus: &mut Bus<T>,
    s: T,
    vx: T,
    preview_time: T,
) {
    bus.set_channel(road.kappa, 0, line.kappa_h(s));
    bus.set_channel(road.grade, 0, line.grade(s));
    bus.set_channel(road.banking, 0, line.banking(s));
    bus.set_channel(road.kappa_v, 0, line.kappa_v(s));
    bus.set_channel(road.n_ref, 0, line.n_ref(s));
    bus.set_channel(road.kappa_ref, 0, line.kappa_ref(s));
    bus.set_channel(road.v_ref, 0, line.v_ref(s));
    // Preview station (line queries wrap/clamp `s + L_p` for closed/open loops).
    let sp = s + preview_distance(vx, preview_time);
    bus.set_channel(road.n_ref_preview, 0, line.n_ref(sp));
    bus.set_channel(road.kappa_ref_preview, 0, line.kappa_ref(sp));
    bus.set_channel(road.v_ref_preview, 0, line.v_ref(sp));
}

/// One full RHS evaluation: clear bus, publish road + preview at `fast`'s `(s, v_x)`, run the block
/// chain in schedule order, leaving the chassis + controller derivatives in `dfast`. Free-standing so
/// callers can hand disjoint field borrows (the RK closure) or `self` fields (the sequential path).
fn eval_rhs_raw<T: Float>(
    blocks: &T2Blocks<T>,
    line: &LineTable<T>,
    fast: &[T],
    dfast: &mut [T],
    bus: &mut Bus<T>,
    torque_scale: T,
    regen_limit_w: T,
) {
    let s = fast[ChassisState::S as usize];
    let vx = fast[ChassisState::Vx as usize];
    bus.clear();
    publish_road(line, &blocks.road, bus, s, vx, blocks.driver.preview_time);
    // Publish the rule-based control-layer boundary values (frozen across the RK sweep): the shift
    // FSM's drive-torque scale and the battery regen ceiling the powertrain reads.
    bus.set_channel(blocks.actuation.torque_scale, 0, torque_scale);
    bus.set_channel(blocks.actuation.regen_limit_w, 0, regen_limit_w);
    let sv = StateView::new(fast, 1, 0);
    let mut dv = DerivView::new(dfast, 1, 0);
    // sense → control → actuate → integrate. The torque-vectoring allocator runs at the tail of the
    // actuate phase, after the powertrain (whose torques it augments) and the tyre/load blocks (whose
    // forces/loads set the friction ellipse), matching the assembler-produced schedule.
    blocks.driver.derivatives(&sv, bus, &mut dv, 0);
    blocks.powertrain.derivatives(&sv, bus, &mut dv, 0);
    blocks.load.derivatives(&sv, bus, &mut dv, 0);
    blocks.aero.derivatives(&sv, bus, &mut dv, 0);
    blocks.tire.derivatives(&sv, bus, &mut dv, 0);
    blocks.tv.derivatives(&sv, bus, &mut dv, 0);
    blocks.chassis.derivatives(&sv, bus, &mut dv, 0);
}

/// The [`ChassisState`] wheel-speed slot for wheel `i`.
#[inline]
fn omega_state(wheel: usize) -> ChassisState {
    match wheel {
        0 => ChassisState::OmegaFl,
        1 => ChassisState::OmegaFr,
        2 => ChassisState::OmegaRl,
        _ => ChassisState::OmegaRr,
    }
}
