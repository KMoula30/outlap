// SPDX-License-Identifier: AGPL-3.0-only
//! The runtime thermal network and its Crank–Nicolson advance.
//!
//! A [`Network`] is `N` nodes with heat capacities, a set of **constant** conductance edges
//! (`g = 1/R`, the conduction/contact skeleton), a set of **convection** edges evaluated each step
//! from the [`crate::correlations`] at the segment's shaft speed and current temperatures, one pinned
//! ambient node, and an optional coolant node closed by a quasi-static jacket balance. Losses are an
//! externally-supplied per-node source; an optional copper-resistance feedback rescales the winding
//! loss with its temperature.
//!
//! The state advances with a semi-implicit trapezoidal (Crank–Nicolson) step
//! `(C/h − G/2)·T₊ = (C/h + G/2)·T + P`, with `G` assembled at the current temperatures. The scheme
//! is A-stable, so a coarse per-segment `h` over a lap stays bounded. The step is allocation-free:
//! `G`, the system matrix, and the solve all live in fixed-size stack buffers ([`MAX_NODES`]).

use crate::correlations::{
    airgap_film, channel_h, churchill_chu_h, endwinding_h, internal_air_h, radiation_h,
    shaft_external_h, FluidProps, Orientation,
};

/// Maximum node count the fixed-size integrator supports (PDT's full network is 20 nodes).
pub const MAX_NODES: usize = 24;

/// Errors from a thermal advance. The QSS caller consumes these as a flagged failure, never a panic.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ThermalError {
    /// The network has more nodes than the fixed-size integrator supports.
    #[error("thermal network has {0} nodes; the integrator supports at most {MAX_NODES}")]
    TooManyNodes(usize),
    /// The Crank–Nicolson system matrix was singular during elimination.
    #[error("the Crank–Nicolson system is singular (pivot {0})")]
    Singular(usize),
    /// A node temperature became non-finite after the step.
    #[error("non-finite temperature after the thermal step")]
    NonFinite,
    /// The step size was not positive.
    #[error("thermal step dt must be > 0, got {0}")]
    BadStep(f64),
}

/// A constant conductance edge `g = 1/R` (W/K) between two nodes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edge {
    /// First node index.
    pub i: usize,
    /// Second node index.
    pub j: usize,
    /// Conductance, W/K.
    pub g_w_per_k: f64,
}

/// The rotor-driven convection law for a cavity/end-winding edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotorAirLaw {
    /// End-winding to internal cavity air (Kylander end-winding form).
    EndWinding,
    /// Internal cavity air to housing inner (Kylander internal-air form).
    InternalAir,
}

/// A convection edge whose conductance is recomputed each step from the correlations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConvKind {
    /// Air-gap film (Becker–Kaye). Depends on shaft speed and the bounding node temperatures.
    AirGap {
        /// Mean air-gap radius, m.
        r_gap_m: f64,
        /// Cold radial gap, m.
        gap0_m: f64,
        /// Iron linear thermal-expansion coefficient, 1/K.
        kappa_fe: f64,
    },
    /// Rotor-driven cavity/end-winding convection; peripheral speed `u = |ω|·r_rotor`.
    RotorAir {
        /// Rotor radius setting the peripheral speed, m.
        r_rotor_m: f64,
        /// Which Kylander form to use.
        law: RotorAirLaw,
    },
    /// Rotating-shaft external convection to ambient (Etemad).
    ShaftExternal {
        /// Shaft diameter, m.
        d_shaft_m: f64,
    },
    /// Liquid-cooled channel (Gnielinski/laminar). Pump-driven, so speed-independent.
    LiquidChannel {
        /// Hydraulic diameter, m.
        hydraulic_diameter_m: f64,
        /// Mean coolant velocity, m/s.
        velocity_mps: f64,
        /// Coolant properties at the film temperature.
        fluid: FluidProps,
    },
    /// Free convection plus linearized radiation to ambient on a cylinder.
    FreeConvection {
        /// Characteristic length, m.
        char_length_m: f64,
        /// Cylinder orientation.
        orientation: Orientation,
        /// Surface emissivity for the radiation term.
        emissivity: f64,
    },
}

/// A convection edge: a node pair, an interface area, and the correlation kind.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConvEdge {
    /// First node index (by convention the solid/stator side for the air-gap film).
    pub i: usize,
    /// Second node index (by convention the rotor/fluid side).
    pub j: usize,
    /// Interface area, m².
    pub area_m2: f64,
    /// The correlation used to turn `(ω, T)` into a conductance.
    pub kind: ConvKind,
}

impl ConvEdge {
    /// Conductance `g = h·A` (W/K) at the given node temperatures, shaft speed, and ambient.
    #[must_use]
    pub fn conductance(&self, t_i_k: f64, t_j_k: f64, omega_rad_s: f64, t_amb_k: f64) -> f64 {
        match self.kind {
            ConvKind::AirGap {
                r_gap_m,
                gap0_m,
                kappa_fe,
            } => {
                // The hotter bounding node is the rotor side (drives the expansion correction).
                let (t_rotor, t_stator) = (t_i_k.max(t_j_k), t_i_k.min(t_j_k));
                let f = airgap_film(
                    omega_rad_s,
                    r_gap_m,
                    gap0_m,
                    t_rotor,
                    t_stator,
                    t_amb_k,
                    kappa_fe,
                );
                f.lam_eff * self.area_m2 / f.delta
            }
            ConvKind::RotorAir { r_rotor_m, law } => {
                let u = omega_rad_s.abs() * r_rotor_m;
                let h = match law {
                    RotorAirLaw::EndWinding => endwinding_h(u),
                    RotorAirLaw::InternalAir => internal_air_h(u),
                };
                h * self.area_m2
            }
            ConvKind::ShaftExternal { d_shaft_m } => {
                shaft_external_h(omega_rad_s, d_shaft_m, t_amb_k) * self.area_m2
            }
            ConvKind::LiquidChannel {
                hydraulic_diameter_m,
                velocity_mps,
                fluid,
            } => channel_h(velocity_mps, hydraulic_diameter_m, fluid) * self.area_m2,
            ConvKind::FreeConvection {
                char_length_m,
                orientation,
                emissivity,
            } => {
                // One side is ambient; the wall is the hotter node.
                let (t_wall, t_cold) = (t_i_k.max(t_j_k), t_i_k.min(t_j_k));
                let h = churchill_chu_h(t_wall, t_cold, char_length_m, orientation)
                    + radiation_h(t_wall, t_cold, emissivity);
                h * self.area_m2
            }
        }
    }
}

/// A coolant node closed by a quasi-static inlet-energy balance instead of integrated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coolant {
    /// Coolant node index.
    pub idx: usize,
    /// Coolant inlet temperature, K.
    pub inlet_k: f64,
    /// Thermal mass-flow capacity `ρ·c_p·ṁ`, W/K.
    pub rho_cp_mdot_w_per_k: f64,
}

/// Copper-resistance feedback: rescale the loss at the listed winding nodes by
/// `1 + α·(T − T_ref)` each step.
#[derive(Clone, Debug, PartialEq)]
pub struct CuFeedback {
    /// Winding node indices whose loss is temperature-scaled.
    pub nodes: Vec<usize>,
    /// Reference temperature, K.
    pub t_ref_k: f64,
    /// Resistance-rise coefficient α, per K.
    pub alpha_per_k: f64,
}

/// A lumped-parameter thermal network. Built once (allocation is fine here); advanced allocation-free.
#[derive(Clone, Debug)]
pub struct Network {
    /// Number of active nodes (≤ [`MAX_NODES`]).
    pub n_nodes: usize,
    /// Heat capacity per node, J/K (the ambient node's entry is unused — it is pinned).
    pub capacity_j_per_k: [f64; MAX_NODES],
    /// The pinned ambient/boundary node index.
    pub ambient_idx: usize,
    /// Optional coolant node.
    pub coolant: Option<Coolant>,
    /// Constant conductance edges (conduction/contact skeleton).
    pub const_edges: Vec<Edge>,
    /// Convection edges recomputed each step.
    pub conv_edges: Vec<ConvEdge>,
    /// Per-node derating limits `(t_warn_c, t_max_c)`; `None` ⇒ the node does not derate.
    pub limits: [Option<(f64, f64)>; MAX_NODES],
    /// Optional copper-resistance feedback.
    pub cu_feedback: Option<CuFeedback>,
}

impl Network {
    /// Advance the node temperatures by one Crank–Nicolson step of `dt_s` seconds.
    ///
    /// `p_base_w[i]` is the base loss injected at node `i` (W), before any copper feedback;
    /// `omega_rad_s` is the source-shaft speed for the convection correlations; `t_amb_k` the pinned
    /// ambient temperature (K). Zero-allocation.
    pub fn advance(
        &self,
        state: &mut ThermalState,
        p_base_w: &[f64],
        omega_rad_s: f64,
        t_amb_k: f64,
        dt_s: f64,
    ) -> Result<(), ThermalError> {
        let n = self.n_nodes;
        if n > MAX_NODES {
            return Err(ThermalError::TooManyNodes(n));
        }
        if dt_s.is_nan() || dt_s <= 0.0 {
            return Err(ThermalError::BadStep(dt_s));
        }
        let t = &state.temp_k;

        // 1. Assemble G at the current temperatures: off-diagonals then the Kirchhoff diagonal.
        let mut g = [[0.0f64; MAX_NODES]; MAX_NODES];
        for e in &self.const_edges {
            g[e.i][e.j] += e.g_w_per_k;
            g[e.j][e.i] += e.g_w_per_k;
        }
        for ce in &self.conv_edges {
            let gg = ce.conductance(t[ce.i], t[ce.j], omega_rad_s, t_amb_k);
            g[ce.i][ce.j] += gg;
            g[ce.j][ce.i] += gg;
        }
        for i in 0..n {
            let mut s = 0.0;
            for j in 0..n {
                if j != i {
                    s += g[i][j];
                }
            }
            g[i][i] = -s;
        }

        // 2. Loss vector with optional copper-resistance feedback.
        let mut p = [0.0f64; MAX_NODES];
        for (i, slot) in p.iter_mut().enumerate().take(n) {
            *slot = p_base_w.get(i).copied().unwrap_or(0.0);
        }
        if let Some(cu) = &self.cu_feedback {
            for &nd in &cu.nodes {
                if nd < n {
                    let factor = (1.0 + cu.alpha_per_k * (t[nd] - cu.t_ref_k)).max(0.0);
                    p[nd] *= factor;
                }
            }
        }

        // 3. Crank–Nicolson system  (C/h − G/2)·T₊ = (C/h + G/2)·T + P.
        let inv_h = 1.0 / dt_s;
        let mut a = [[0.0f64; MAX_NODES]; MAX_NODES];
        let mut b = [0.0f64; MAX_NODES];
        for i in 0..n {
            for j in 0..n {
                a[i][j] = -0.5 * g[i][j];
            }
            a[i][i] += self.capacity_j_per_k[i] * inv_h;

            let mut bi = p[i];
            for j in 0..n {
                let c_ij = if i == j {
                    self.capacity_j_per_k[i] * inv_h
                } else {
                    0.0
                };
                bi += (c_ij + 0.5 * g[i][j]) * t[j];
            }
            b[i] = bi;
        }

        // 4. Pin the ambient row to T_amb.
        let ia = self.ambient_idx;
        for j in 0..n {
            a[ia][j] = 0.0;
        }
        a[ia][ia] = 1.0;
        b[ia] = t_amb_k;

        // 5. Coolant row: quasi-static jacket balance  T_c = inlet + Q_in / (2·ρc_pṁ).
        if let Some(c) = self.coolant {
            let mut q_in = 0.0;
            for j in 0..n {
                if j != c.idx && j != ia {
                    q_in += g[c.idx][j] * (t[j] - t[c.idx]);
                }
            }
            let target = c.inlet_k + q_in / (2.0 * c.rho_cp_mdot_w_per_k.max(1e-9));
            for j in 0..n {
                a[c.idx][j] = 0.0;
            }
            a[c.idx][c.idx] = 1.0;
            b[c.idx] = target;
        }

        // 6. Solve, commit, re-pin ambient, guard finiteness.
        let mut x = b;
        solve_in_place(&mut a, &mut x, n)?;
        for i in 0..n {
            if !x[i].is_finite() {
                return Err(ThermalError::NonFinite);
            }
            state.temp_k[i] = x[i];
        }
        state.temp_k[ia] = t_amb_k;
        Ok(())
    }

    /// The commanded-torque derating factor in `[0, 1]` — the minimum over all limited nodes of a
    /// linear ramp `1 → 0` across `T_warn → T_max` (the winding normally binds, §8.5).
    #[must_use]
    pub fn derate(&self, state: &ThermalState) -> f64 {
        let mut d = 1.0f64;
        for i in 0..self.n_nodes {
            if let Some((warn, max)) = self.limits[i] {
                let t_c = state.temp_k[i] - 273.15;
                let f = if max <= warn {
                    f64::from(t_c < max)
                } else {
                    ((max - t_c) / (max - warn)).clamp(0.0, 1.0)
                };
                d = d.min(f);
            }
        }
        d
    }
}

/// The mutable node-temperature state of a network.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermalState {
    /// Number of active nodes.
    pub n_nodes: usize,
    /// Node temperatures, K (only the first `n_nodes` entries are meaningful).
    pub temp_k: [f64; MAX_NODES],
}

impl ThermalState {
    /// All nodes at a uniform temperature (K).
    #[must_use]
    pub fn uniform(n_nodes: usize, t_k: f64) -> Self {
        let mut temp_k = [t_k; MAX_NODES];
        for slot in temp_k.iter_mut().skip(n_nodes) {
            *slot = t_k;
        }
        Self { n_nodes, temp_k }
    }

    /// From an explicit list of node temperatures (K).
    #[must_use]
    pub fn from_temps(temps: &[f64]) -> Self {
        let mut temp_k = [0.0f64; MAX_NODES];
        for (slot, &v) in temp_k.iter_mut().zip(temps) {
            *slot = v;
        }
        Self {
            n_nodes: temps.len(),
            temp_k,
        }
    }

    /// Node temperature in °C.
    #[must_use]
    pub fn temp_c(&self, node: usize) -> f64 {
        self.temp_k[node] - 273.15
    }
}

/// Gauss elimination with partial pivoting, in place on `a` and the RHS `x` (`n×n`). Zero-allocation.
fn solve_in_place(
    a: &mut [[f64; MAX_NODES]; MAX_NODES],
    x: &mut [f64; MAX_NODES],
    n: usize,
) -> Result<(), ThermalError> {
    for k in 0..n {
        // Partial pivot.
        let mut piv = k;
        let mut best = a[k][k].abs();
        for r in (k + 1)..n {
            let v = a[r][k].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if best < 1e-30 {
            return Err(ThermalError::Singular(k));
        }
        if piv != k {
            a.swap(piv, k);
            x.swap(piv, k);
        }
        // Eliminate below.
        let akk = a[k][k];
        for r in (k + 1)..n {
            let f = a[r][k] / akk;
            if f != 0.0 {
                for c in k..n {
                    a[r][c] -= f * a[k][c];
                }
                x[r] -= f * x[k];
            }
        }
    }
    // Back-substitute (x holds the reduced RHS, overwritten with the solution top-down from the end).
    for i in (0..n).rev() {
        let mut s = x[i];
        for j in (i + 1)..n {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    /// A single integrated node (index 0) coupled to ambient (index 1) by a constant g.
    fn two_node(cap: f64, g: f64, t_amb_k: f64) -> (Network, ThermalState) {
        let mut capacity = [0.0; MAX_NODES];
        capacity[0] = cap;
        let mut limits = [None; MAX_NODES];
        limits[0] = Some((150.0, 180.0));
        let net = Network {
            n_nodes: 2,
            capacity_j_per_k: capacity,
            ambient_idx: 1,
            coolant: None,
            const_edges: vec![Edge {
                i: 0,
                j: 1,
                g_w_per_k: g,
            }],
            conv_edges: vec![],
            limits,
            cu_feedback: None,
        };
        (net, ThermalState::uniform(2, t_amb_k))
    }

    #[test]
    fn lti_first_order_matches_analytic() {
        // C·Ṫ = P − g·(T − T_amb): T(t) = T_amb + (P/g)(1 − e^{−t g/C}).
        let (cap, g, t_amb, p) = (2000.0, 5.0, 300.0, 1500.0);
        let (net, mut st) = two_node(cap, g, t_amb);
        // τ = C/g = 400 s; run ≈40·τ so the exponential transient is fully decayed.
        let dt = 0.5;
        let n_steps = 32_000;
        for _ in 0..n_steps {
            net.advance(&mut st, &[p, 0.0], 0.0, t_amb, dt).unwrap();
        }
        let t_ss = t_amb + p / g;
        assert!(
            (st.temp_k[0] - t_ss).abs() < 1e-3,
            "T={} vs ss={}",
            st.temp_k[0],
            t_ss
        );

        // Mid-transient point against the closed form.
        let (net, mut st) = two_node(cap, g, t_amb);
        let tau = cap / g;
        let t_target = 0.7 * tau;
        let dt = t_target / 500.0;
        for _ in 0..500 {
            net.advance(&mut st, &[p, 0.0], 0.0, t_amb, dt).unwrap();
        }
        let analytic = t_amb + (p / g) * (1.0 - (-t_target / tau).exp());
        assert!(
            (st.temp_k[0] - analytic).abs() < 0.5,
            "num={} analytic={}",
            st.temp_k[0],
            analytic
        );
    }

    #[test]
    fn ambient_node_stays_pinned() {
        let (net, mut st) = two_node(1000.0, 3.0, 295.0);
        net.advance(&mut st, &[800.0, 0.0], 0.0, 295.0, 1.0)
            .unwrap();
        assert_eq!(st.temp_k[1], 295.0);
    }

    #[test]
    fn derate_is_monotone_in_temperature() {
        let (net, _) = two_node(1000.0, 3.0, 300.0);
        let mut cool = ThermalState::uniform(2, 300.0);
        cool.temp_k[0] = 273.15 + 140.0; // below warn
        let mut warm = ThermalState::uniform(2, 300.0);
        warm.temp_k[0] = 273.15 + 165.0; // between warn and max
        let mut hot = ThermalState::uniform(2, 300.0);
        hot.temp_k[0] = 273.15 + 185.0; // above max
        assert_eq!(net.derate(&cool), 1.0);
        assert!(net.derate(&warm) > 0.0 && net.derate(&warm) < 1.0);
        assert_eq!(net.derate(&hot), 0.0);
        assert!(net.derate(&warm) < net.derate(&cool));
    }

    #[test]
    fn stint_heat_soak_is_monotone() {
        // Repeated constant-loss "laps" from an ambient start: winding temperature rises each lap
        // and the derate never increases.
        let (net, mut st) = two_node(3000.0, 4.0, 300.0);
        let mut last_t = st.temp_k[0];
        let mut last_d = 1.0;
        for _ in 0..20 {
            for _ in 0..100 {
                net.advance(&mut st, &[2500.0, 0.0], 0.0, 300.0, 0.1)
                    .unwrap();
            }
            assert!(st.temp_k[0] >= last_t - 1e-9);
            let d = net.derate(&st);
            assert!(d <= last_d + 1e-9);
            last_t = st.temp_k[0];
            last_d = d;
        }
    }

    #[test]
    fn coolant_node_holds_quasi_static_target() {
        // Node 0 (hot solid) → coolant node 1 → ambient node 2. The coolant sits at
        // inlet + Q_in/(2·ρc_pṁ) once node 0 is warm.
        let mut capacity = [0.0; MAX_NODES];
        capacity[0] = 1000.0;
        let net = Network {
            n_nodes: 3,
            capacity_j_per_k: capacity,
            ambient_idx: 2,
            coolant: Some(Coolant {
                idx: 1,
                inlet_k: 330.0,
                rho_cp_mdot_w_per_k: 500.0,
            }),
            const_edges: vec![Edge {
                i: 0,
                j: 1,
                g_w_per_k: 20.0,
            }],
            conv_edges: vec![],
            limits: [None; MAX_NODES],
            cu_feedback: None,
        };
        let mut st = ThermalState::from_temps(&[300.0, 330.0, 300.0]);
        for _ in 0..5000 {
            net.advance(&mut st, &[1000.0, 0.0, 0.0], 0.0, 300.0, 0.5)
                .unwrap();
        }
        // At steady state all injected power crosses g(0→coolant): Q_in = P = 1000 W.
        let expected = 330.0 + 1000.0 / (2.0 * 500.0);
        assert!(
            (st.temp_k[1] - expected).abs() < 1e-2,
            "coolant={} exp={}",
            st.temp_k[1],
            expected
        );
    }

    #[test]
    fn cu_feedback_raises_steady_temperature() {
        // Copper feedback increases dissipation as the winding heats, so steady T exceeds the
        // no-feedback case.
        let base = {
            let (net, mut st) = two_node(2000.0, 5.0, 300.0);
            for _ in 0..4000 {
                net.advance(&mut st, &[1500.0, 0.0], 0.0, 300.0, 0.5)
                    .unwrap();
            }
            st.temp_k[0]
        };
        let (mut net, mut st) = two_node(2000.0, 5.0, 300.0);
        net.cu_feedback = Some(CuFeedback {
            nodes: vec![0],
            t_ref_k: 300.0,
            alpha_per_k: 0.00393,
        });
        for _ in 0..4000 {
            net.advance(&mut st, &[1500.0, 0.0], 0.0, 300.0, 0.5)
                .unwrap();
        }
        assert!(
            st.temp_k[0] > base + 1.0,
            "fb={} base={}",
            st.temp_k[0],
            base
        );
    }
}
