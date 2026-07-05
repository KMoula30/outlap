// SPDX-License-Identifier: AGPL-3.0-only
//! Assembly of an `.emotor` document into a runnable [`MachineThermal`] and its per-segment advance.
//!
//! This is the cold path between the schema layer ([`outlap_schema::emotor::Emotor`]) and the runtime
//! network ([`outlap_thermal::Network`]). Node names become indices; omitted heat capacities and
//! conductances on a lumped model are filled from documented mass heuristics (flagged as estimates in
//! the loaded-model report); convection edges map to their correlation kinds; the loss-routing table
//! resolves to node indices. The advance ([`MachineThermal::step`]) builds the per-node loss vector at
//! an operating point and takes one Crank–Nicolson step, returning the torque derate.
//!
//! **Loss rule (user decision 2026-07-05):** the `.ptm` supplies the total machine-heating loss;
//! declared routes deposit their share; whatever total is not routed lands on the winding node (so a
//! model with no explicit routing puts all loss into the winding — the conservative default).

use outlap_schema::conditions::Conditions;
use outlap_schema::emotor::{
    AirGapSpec, ConvModel, CoolantProps, Emotor, FluidSpec, InitialTemp, JacketSpec, NodeRole,
};
use outlap_thermal::{
    ConvEdge, ConvKind, Coolant, CuFeedback, Edge, Network, RotorAirLaw, ThermalError,
    ThermalState, MAX_NODES,
};

use crate::error::T1Error;

/// Absolute zero offset for the °C↔K boundary (SI internally; °C only at the file boundary).
const CELSIUS_K: f64 = 273.15;
/// Default iron linear thermal-expansion coefficient, 1/K (electrical steel).
const KAPPA_FE: f64 = 10.4e-6;

/// Which `.ptm` loss quantity a routing entry draws from.
#[derive(Clone, Debug, PartialEq, Eq)]
enum LossSource {
    /// The total machine loss (`loss_w`) supplied to [`MachineThermal::step`].
    Total,
    /// A named per-component loss column, looked up via the caller's closure.
    Named(String),
}

/// A resolved loss-routing entry.
#[derive(Clone, Debug)]
struct Route {
    source: LossSource,
    node: usize,
    fraction: f64,
}

/// A machine thermal model assembled from an `.emotor` document: the network, its live state, the
/// resolved loss routing, and the loaded-model estimate notes.
#[derive(Clone, Debug)]
pub struct MachineThermal {
    net: Network,
    state: ThermalState,
    names: Vec<String>,
    routes: Vec<Route>,
    winding_idx: usize,
    ambient_k: f64,
    estimates: Vec<String>,
}

impl MachineThermal {
    /// Assemble from an `.emotor` document, the session ambient (°C, from `conditions.yaml`), and the
    /// machine mass (kg, from the `.ptm`) used by the capacity/conductance heuristics.
    #[allow(clippy::too_many_lines)] // one linear assembly procedure; splitting it hurts clarity.
    pub fn assemble(
        emotor: &Emotor,
        conditions: &Conditions,
        machine_mass_kg: f64,
    ) -> Result<Self, T1Error> {
        let n = emotor.nodes.len();
        if n > MAX_NODES {
            return Err(T1Error::Thermal(format!(
                "{n} nodes exceeds the integrator limit of {MAX_NODES}"
            )));
        }
        let names: Vec<String> = emotor.nodes.iter().map(|nd| nd.name.clone()).collect();
        let index_of = |name: &str| -> Result<usize, T1Error> {
            names
                .iter()
                .position(|x| x == name)
                .ok_or_else(|| T1Error::Thermal(format!("unknown node `{name}`")))
        };

        let ambient_idx = index_of(&emotor.cooling.ambient_node)?;
        // The coolant node (if any) is closed by the quasi-static balance, not integrated, so its
        // capacity is unused — like the ambient node it needs neither an explicit value nor a
        // heuristic. It may be declared by the low-level `coolant` spec or a `jacket` block.
        let coolant_name = emotor
            .cooling
            .coolant
            .as_ref()
            .map(|c| c.node.as_str())
            .or_else(|| {
                emotor
                    .cooling
                    .jacket
                    .as_ref()
                    .map(|j| j.coolant_node.as_str())
            });
        let coolant_idx = coolant_name.map(index_of).transpose()?;
        let mut estimates: Vec<String> = Vec::new();

        // Heat capacities: explicit, else a mass heuristic (flagged), else an error.
        let mut capacity_j_per_k = [0.0f64; MAX_NODES];
        for (i, nd) in emotor.nodes.iter().enumerate() {
            if i == ambient_idx || Some(i) == coolant_idx {
                continue; // pinned / balance-closed; capacity unused
            }
            if let Some(c) = nd.c_j_per_k {
                capacity_j_per_k[i] = c;
            } else if let Some(c) = nd.role.and_then(|r| capacity_heuristic(r, machine_mass_kg)) {
                capacity_j_per_k[i] = c;
                estimates.push(format!(
                    "node `{}` capacity estimated from mass ({c:.0} J/K)",
                    nd.name
                ));
            } else {
                return Err(T1Error::Thermal(format!(
                    "node `{}` has no `c_j_per_k` and no mass heuristic for its role — set it explicitly",
                    nd.name
                )));
            }
        }

        // Per-node derating limits.
        let mut limits = [None; MAX_NODES];
        for (i, nd) in emotor.nodes.iter().enumerate() {
            if let (Some(w), Some(m)) = (nd.t_warn_c, nd.t_max_c) {
                limits[i] = Some((w, m));
            }
        }

        // Constant conductance edges: explicit, else a role-pair heuristic (flagged), else an error.
        let mut const_edges = Vec::with_capacity(emotor.conductances.len());
        for edge in &emotor.conductances {
            let i = index_of(&edge.between.0)?;
            let j = index_of(&edge.between.1)?;
            let g = if let Some(g) = edge.w_per_k {
                g
            } else {
                let (ra, rb) = (emotor.nodes[i].role, emotor.nodes[j].role);
                match ra
                    .zip(rb)
                    .and_then(|(a, b)| conductance_heuristic(a, b, machine_mass_kg))
                {
                    Some(g) => {
                        estimates.push(format!(
                            "conductance `{}`↔`{}` estimated from mass ({g:.1} W/K)",
                            edge.between.0, edge.between.1
                        ));
                        g
                    }
                    None => {
                        return Err(T1Error::Thermal(format!(
                            "conductance `{}`↔`{}` has no `w_per_k` and no mass heuristic for these roles",
                            edge.between.0, edge.between.1
                        )));
                    }
                }
            };
            const_edges.push(Edge { i, j, g_w_per_k: g });
        }

        // Convection edges → correlation kinds.
        let mut conv_edges = Vec::with_capacity(emotor.convection.len());
        for ce in &emotor.convection {
            let i = index_of(&ce.between.0)?;
            let j = index_of(&ce.between.1)?;
            let kind = match &ce.model {
                ConvModel::AirGap {
                    r_gap_m,
                    gap0_m,
                    kappa_fe,
                } => ConvKind::AirGap {
                    r_gap_m: *r_gap_m,
                    gap0_m: *gap0_m,
                    kappa_fe: kappa_fe.unwrap_or(KAPPA_FE),
                },
                ConvModel::RotorAir { r_rotor_m, law } => ConvKind::RotorAir {
                    r_rotor_m: *r_rotor_m,
                    law: match law {
                        outlap_schema::emotor::RotorAirLaw::EndWinding => RotorAirLaw::EndWinding,
                        outlap_schema::emotor::RotorAirLaw::InternalAir => RotorAirLaw::InternalAir,
                    },
                },
                ConvModel::ShaftExternal { d_shaft_m } => ConvKind::ShaftExternal {
                    d_shaft_m: *d_shaft_m,
                },
                ConvModel::LiquidChannel {
                    hydraulic_diameter_m,
                    velocity_mps,
                    fluid,
                } => ConvKind::LiquidChannel {
                    hydraulic_diameter_m: *hydraulic_diameter_m,
                    velocity_mps: *velocity_mps,
                    fluid: outlap_thermal::FluidProps {
                        lam: fluid.lam,
                        nu: fluid.nu,
                        pr: fluid.pr,
                    },
                },
                ConvModel::FreeConvection {
                    char_length_m,
                    orientation,
                    emissivity,
                } => ConvKind::FreeConvection {
                    char_length_m: *char_length_m,
                    orientation: match orientation {
                        outlap_schema::emotor::Orientation::Horizontal => {
                            outlap_thermal::Orientation::Horizontal
                        }
                        outlap_schema::emotor::Orientation::Vertical => {
                            outlap_thermal::Orientation::Vertical
                        }
                    },
                    emissivity: *emissivity,
                },
            };
            conv_edges.push(ConvEdge {
                i,
                j,
                area_m2: ce.area_m2,
                kind,
            });
        }

        // Coolant node — from the low-level `coolant` spec or derived from a `jacket` block.
        let mut coolant = match &emotor.cooling.coolant {
            Some(c) => Some(Coolant {
                idx: index_of(&c.node)?,
                inlet_k: c.inlet_c + CELSIUS_K,
                rho_cp_mdot_w_per_k: c.rho_cp_mdot_w_per_k,
            }),
            None => None,
        };
        if let Some(j) = &emotor.cooling.jacket {
            if coolant.is_some() {
                return Err(T1Error::Thermal(
                    "declare either `cooling.coolant` or `cooling.jacket`, not both".into(),
                ));
            }
            let (edge, cool) = derive_jacket(j, &index_of)?;
            conv_edges.push(edge);
            coolant = Some(cool);
        }
        if let Some(a) = &emotor.cooling.air_gap {
            conv_edges.push(derive_air_gap(a, &index_of)?);
        }

        // Copper feedback.
        let cu_feedback = match &emotor.cu_feedback {
            Some(cu) => {
                let mut nodes = Vec::with_capacity(cu.nodes.len());
                for name in &cu.nodes {
                    nodes.push(index_of(name)?);
                }
                Some(CuFeedback {
                    nodes,
                    t_ref_k: cu.t_ref_c + CELSIUS_K,
                    alpha_per_k: cu.alpha_per_k,
                })
            }
            None => None,
        };

        // Loss routing + the winding node (remainder target).
        let winding_idx = emotor
            .nodes
            .iter()
            .position(|nd| nd.role == Some(NodeRole::Winding))
            .ok_or_else(|| T1Error::Thermal("no node has role `winding`".into()))?;
        let mut routes = Vec::with_capacity(emotor.loss_routing.len());
        for r in &emotor.loss_routing {
            routes.push(Route {
                source: r
                    .component
                    .clone()
                    .map_or(LossSource::Total, LossSource::Named),
                node: index_of(&r.node)?,
                fraction: r.fraction,
            });
        }

        // Ambient temperature: explicit override, else the session ambient.
        let ambient_k = emotor
            .cooling
            .ambient_fixed_c
            .unwrap_or(conditions.ambient_c)
            + CELSIUS_K;

        let net = Network {
            n_nodes: n,
            capacity_j_per_k,
            ambient_idx,
            coolant,
            const_edges,
            conv_edges,
            limits,
            cu_feedback,
        };

        // Initial temperatures: sink by default (ambient, coolant inlet for the coolant node).
        let mut temp_k = [ambient_k; MAX_NODES];
        if let Some(c) = net.coolant {
            temp_k[c.idx] = c.inlet_k;
        }
        match &emotor.initial_temp {
            None => {}
            Some(InitialTemp::UniformC(t)) => {
                for slot in temp_k.iter_mut().take(n) {
                    *slot = t + CELSIUS_K;
                }
            }
            Some(InitialTemp::PerNodeC(list)) => {
                for nt in list {
                    temp_k[index_of(&nt.node)?] = nt.temp_c + CELSIUS_K;
                }
            }
        }
        temp_k[ambient_idx] = ambient_k;
        let state = ThermalState { n_nodes: n, temp_k };

        Ok(Self {
            net,
            state,
            names,
            routes,
            winding_idx,
            ambient_k,
            estimates,
        })
    }

    /// Advance one segment. `machine_loss_w` is the total machine-heating loss at the operating point
    /// (inverter excluded); `component_loss` resolves named per-component columns (return `None` for
    /// an absent column). `omega_rad_s` is the source-shaft speed. Returns the torque derate factor.
    pub fn step(
        &mut self,
        machine_loss_w: f64,
        component_loss: impl Fn(&str) -> Option<f64>,
        omega_rad_s: f64,
        dt_s: f64,
    ) -> Result<f64, ThermalError> {
        let mut p = [0.0f64; MAX_NODES];
        let mut deposited = 0.0;
        for r in &self.routes {
            let value = match &r.source {
                LossSource::Total => machine_loss_w,
                LossSource::Named(name) => component_loss(name).unwrap_or(0.0),
            };
            let dep = value * r.fraction;
            p[r.node] += dep;
            deposited += dep;
        }
        // Whatever total loss is not routed lands on the winding node (never remove heat).
        let remainder = machine_loss_w - deposited;
        if remainder > 0.0 {
            p[self.winding_idx] += remainder;
        }
        self.net
            .advance(&mut self.state, &p, omega_rad_s, self.ambient_k, dt_s)?;
        Ok(self.net.derate(&self.state))
    }

    /// The current torque derating factor in `[0, 1]`.
    #[must_use]
    pub fn derate(&self) -> f64 {
        self.net.derate(&self.state)
    }

    /// Temperature of the **winding** node, °C — the rated node that normally binds the derate and
    /// the representative machine temperature the QSS slow-state coupling logs per segment.
    #[must_use]
    pub fn winding_temp_c(&self) -> f64 {
        self.state.temp_k[self.winding_idx] - CELSIUS_K
    }

    /// Temperature of a node by name, °C (`None` if the name is unknown).
    #[must_use]
    pub fn temp_c(&self, node: &str) -> Option<f64> {
        self.names
            .iter()
            .position(|x| x == node)
            .map(|i| self.state.temp_k[i] - CELSIUS_K)
    }

    /// The node names, in index order (result-channel labels).
    #[must_use]
    pub fn node_names(&self) -> &[String] {
        &self.names
    }

    /// Loaded-model report notes for every heuristic-filled value.
    #[must_use]
    pub fn estimates(&self) -> &[String] {
        &self.estimates
    }
}

/// Mass-based heat-capacity heuristic (documented rough estimate, Decision #25): `C = f·m·c_p`.
#[must_use]
fn capacity_heuristic(role: NodeRole, mass_kg: f64) -> Option<f64> {
    let (frac, cp) = match role {
        NodeRole::Winding => (0.15, 385.0),    // copper
        NodeRole::StatorIron => (0.45, 460.0), // electrical steel
        NodeRole::Rotor => (0.25, 450.0),      // steel + magnet
        NodeRole::Housing => (0.15, 900.0),    // aluminium
        NodeRole::Coolant | NodeRole::Ambient | NodeRole::Other => return None,
    };
    Some(frac * mass_kg * cp)
}

/// Mass-based conductance heuristic for a role pair (documented rough estimate): a reference value at
/// `m₀ = 40 kg` scaled by `(m/m₀)^{2/3}` (interface area ∝ mass^{2/3}). `None` ⇒ author it explicitly.
#[must_use]
fn conductance_heuristic(a: NodeRole, b: NodeRole, mass_kg: f64) -> Option<f64> {
    use NodeRole::{Ambient, Coolant, Housing, Rotor, StatorIron, Winding};
    let scale = (mass_kg / 40.0).powf(2.0 / 3.0);
    let pair = |r1, r2| (a == r1 && b == r2) || (a == r2 && b == r1);
    let base = if pair(Winding, StatorIron) {
        30.0
    } else if pair(Winding, Housing) {
        8.0
    } else if pair(StatorIron, Housing) {
        60.0
    } else if pair(Housing, Coolant) {
        200.0
    } else if pair(Housing, Ambient) {
        5.0
    } else if pair(Rotor, StatorIron) || pair(Rotor, Housing) || pair(Rotor, Winding) {
        3.0
    } else {
        return None;
    };
    Some(base * scale)
}

/// Millimetres → metres.
const MM: f64 = 1.0e-3;

/// Resolve fluid properties from a named preset or explicit values (documented coolant table).
fn fluid_props(spec: &FluidSpec) -> Result<CoolantProps, T1Error> {
    match spec {
        FluidSpec::Props(p) => Ok(*p),
        FluidSpec::Named(name) => match name.as_str() {
            // ρ [kg/m³], c_p [J/kg·K], λ [W/m·K], ν [m²/s], Pr — at ~60–70 °C film.
            "water" => Ok(CoolantProps {
                rho: 983.0,
                cp: 4185.0,
                lam: 0.654,
                nu: 4.7e-7,
                pr: 3.0,
            }),
            "ethylene_glycol_50" => Ok(CoolantProps {
                rho: 1043.0,
                cp: 3450.0,
                lam: 0.401,
                nu: 1.74e-6,
                pr: 15.6,
            }),
            "oil" => Ok(CoolantProps {
                rho: 860.0,
                cp: 2000.0,
                lam: 0.14,
                nu: 2.0e-5,
                pr: 280.0,
            }),
            other => Err(T1Error::Thermal(format!(
                "unknown coolant fluid `{other}` (known: water, ethylene_glycol_50, oil) — or give \
                 explicit `props`"
            ))),
        },
    }
}

/// Derive the `housing↔coolant` channel-convection edge and the coolant node from a jacket spec.
fn derive_jacket(
    j: &JacketSpec,
    index_of: &impl Fn(&str) -> Result<usize, T1Error>,
) -> Result<(ConvEdge, Coolant), T1Error> {
    let housing = index_of(&j.housing_node)?;
    let coolant_idx = index_of(&j.coolant_node)?;
    let fluid = fluid_props(&j.fluid)?;
    let (w, h) = (j.channel_width_mm * MM, j.channel_height_mm * MM);
    let q_m3s = j.flow_rate_lps * 1.0e-3;
    let n = f64::from(j.channel_count.max(1));
    let a_cross = w * h;
    // Mean channel velocity, hydraulic diameter (rectangular duct), coolant capacity rate ρ·c_p·ṁ.
    let velocity = q_m3s / (n * a_cross.max(1e-12));
    let hydraulic_diameter = 2.0 * w * h / (w + h).max(1e-9);
    let rho_cp_mdot = fluid.cp * fluid.rho * q_m3s; // c_p·ṁ, with ṁ = ρ·Q
    let edge = ConvEdge {
        i: housing,
        j: coolant_idx,
        area_m2: j.wetted_area_m2,
        kind: ConvKind::LiquidChannel {
            hydraulic_diameter_m: hydraulic_diameter,
            velocity_mps: velocity,
            fluid: outlap_thermal::FluidProps {
                lam: fluid.lam,
                nu: fluid.nu,
                pr: fluid.pr,
            },
        },
    };
    let coolant = Coolant {
        idx: coolant_idx,
        inlet_k: j.inlet_c + CELSIUS_K,
        rho_cp_mdot_w_per_k: rho_cp_mdot,
    };
    Ok((edge, coolant))
}

/// Derive an air-gap film edge from raw rotor geometry: `r_gap = r_ro + gap/2`, `A = 2π·r_gap·L`.
fn derive_air_gap(
    a: &AirGapSpec,
    index_of: &impl Fn(&str) -> Result<usize, T1Error>,
) -> Result<ConvEdge, T1Error> {
    let i = index_of(&a.between.0)?;
    let j = index_of(&a.between.1)?;
    let (r_ro, gap, l) = (
        a.rotor_outer_radius_mm * MM,
        a.gap_mm * MM,
        a.stack_length_mm * MM,
    );
    let r_gap = r_ro + 0.5 * gap;
    let area = 2.0 * std::f64::consts::PI * r_gap * l;
    Ok(ConvEdge {
        i,
        j,
        area_m2: area,
        kind: ConvKind::AirGap {
            r_gap_m: r_gap,
            gap0_m: gap,
            kappa_fe: KAPPA_FE,
        },
    })
}
