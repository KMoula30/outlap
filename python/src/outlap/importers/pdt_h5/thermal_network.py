# SPDX-License-Identifier: AGPL-3.0-only
"""Build a detailed ``emotor/1.1`` network by aggregating a PDT LPTN into the reduced node menu.

The PDT ``thermal_obj`` carries per-node capacities ``C`` and a constant conduction matrix
``G_const``, but the heat-rejection paths (air-gap film, coolant jacket) are *convection* — they are
NOT in ``G_const`` and are rebuilt from clean scalar fields (``info/air_gap_mm``,
``info/rotor/outer_radius_mm``, stack length, and ``thermal_obj/user/cooling_liquid_jacket`` /
``thermal_obj/cooling``), never the FEA mesh (§8.5, Decision #25 as amended).

So the faithful import collapses the 20-node network onto the reduced menu — winding / stator_iron /
rotor / housing / coolant / ambient — summing the real capacities and inter-group conductances, then
attaching the derived air-gap and jacket cooling blocks. outlap evaluates the correlations per segment.
"""

from __future__ import annotations

from typing import Any

import h5py
import numpy as np

from . import common as c

# Which detailed node each reduced group collapses (names as PDT emits them).
NODE_GROUPS: dict[str, list[str]] = {
    "winding": ["slot_active", "end_winding_de", "end_winding_nde"],
    "stator_iron": [
        "stator_yoke_upper",
        "stator_yoke_lower",
        "stator_tooth_body",
        "stator_tooth_tip",
    ],
    "rotor": ["magnet", "rotor_bridge", "rotor_bridge_below", "shaft", "airgap"],
    "housing": [
        "housing_body",
        "housing_outer",
        "bearing_de",
        "bearing_nde",
        "end_cavity_de",
        "end_cavity_nde",
    ],
    "coolant": ["coolant"],
    "ambient": ["ambient"],
}
_ROLE = {
    "winding": "winding",
    "stator_iron": "stator_iron",
    "rotor": "rotor",
    "housing": "housing",
    "coolant": "coolant",
    "ambient": "ambient",
}
# loss_breakdown component → reduced group.
LOSS_TO_GROUP: dict[str, str] = {
    "winding_active": "winding",
    "winding_end_turn": "winding",
    "winding_stator": "winding",
    "core_stator": "stator_iron",
    "stator_yoke_upper": "stator_iron",
    "stator_yoke_lower": "stator_iron",
    "stator_tooth_body": "stator_iron",
    "stator_tooth_tip": "stator_iron",
    "core_rotor": "rotor",
    "magnet": "rotor",
    "rotor_bridge": "rotor",
    "rotor_bridge_below": "rotor",
    "shaft": "rotor",
    "housing": "housing",
    "bearing_total": "housing",
    "mechanical": "housing",
}


def _group_of(node: str) -> str | None:
    for grp, members in NODE_GROUPS.items():
        if node in members:
            return grp
    return None


def _decode_name(raw: Any) -> str:
    """Decode an h5 node name, unwrapping the DriveUnit double-encoding ``b"b'ambient'"`` → ``ambient``."""
    s = raw.decode() if isinstance(raw, bytes) else str(raw)
    # Strip up to two layers of Python bytes-literal quoting (a known DriveUnit export bug).
    for _ in range(2):
        if s.startswith("b'") and s.endswith("'"):
            s = s[2:-1]
        elif s.startswith('b"') and s.endswith('"'):
            s = s[2:-1]
    return s


def read_detailed_thermal(f: h5py.File) -> dict[str, Any] | None:
    """Read the scalars for a detailed import from an open PDT `.h5`, or `None` if unavailable.

    Reads the LPTN (``thermal_obj/C`` + ``G_const`` + ``node_names``), the air-gap geometry
    (``info/air_gap_mm``, ``info/rotor/outer_radius_mm``, stack length), the jacket settings
    (``thermal_obj/user/cooling_liquid_jacket`` + derived ``cooling`` values), the temperature limits,
    and the per-component loss split (aggregated to reduced groups). Never reads the FEA mesh.
    """
    # All h5 access routes through the typed `common` helpers (opt_arr/scalar) so no raw indexing.
    names_arr = c.opt_arr(f, "thermal_obj/node_names")
    c_arr = c.opt_arr(f, "thermal_obj/C")
    g_arr = c.opt_arr(f, "thermal_obj/G_const")
    flow = c.opt_arr(f, "thermal_obj/user/cooling_liquid_jacket/coolant_flow_rate")
    if names_arr is None or c_arr is None or g_arr is None or flow is None:
        return None
    names = [_decode_name(x) for x in names_arr]
    # Need at least the reduced-menu members to aggregate meaningfully.
    if not any(_group_of(n) == "winding" for n in names):
        return None
    c_vec = np.asarray(c_arr, dtype=float)
    g_const = np.asarray(g_arr, dtype=float)

    sc = c.scalar
    air_gap_mm = sc(
        f, "info/air_gap_mm", 1000.0 * sc(f, "thermal_obj/rotor_dims/air_gap_m", 0.001)
    )
    r_ro_mm = sc(
        f,
        "info/rotor/outer_radius_mm",
        1000.0 * sc(f, "thermal_obj/rotor_dims/r_ro", 0.075),
    )
    stack_mm = 1000.0 * sc(
        f, "thermal_obj/stator_dims/L", sc(f, "info/active_length", 0.12)
    )

    # Coolant fluid: explicit properties from the derived `cooling/fluid_props`, else a named default.
    rho = c.opt_arr(f, "thermal_obj/cooling/fluid_props/rho")
    fluid: dict[str, Any]
    if rho is not None:
        fluid = {
            "props": {
                "rho": round(float(rho[()]), 2),
                "cp": round(sc(f, "thermal_obj/cooling/fluid_props/cp", 3450.0), 2),
                "lam": round(sc(f, "thermal_obj/cooling/fluid_props/lam", 0.401), 4),
                "nu": round(sc(f, "thermal_obj/cooling/fluid_props/nu", 1.74e-6), 10),
                "pr": round(sc(f, "thermal_obj/cooling/fluid_props/pr", 15.6), 3),
            }
        }
    else:
        fluid = {"named": "ethylene_glycol_50"}
    jacket = {
        "inlet_c": sc(f, "thermal_obj/cooling/coolant_inlet_K", 338.15) - 273.15,
        "flow_rate_lps": 1000.0 * float(np.asarray(flow).reshape(-1)[0]),
        "channel_count": int(
            sc(f, "thermal_obj/user/cooling_liquid_jacket/channel_count", 1.0)
        ),
        "channel_width_mm": sc(
            f, "thermal_obj/user/cooling_liquid_jacket/channel_width", 8.0
        ),
        "channel_height_mm": sc(
            f, "thermal_obj/user/cooling_liquid_jacket/channel_height", 10.0
        ),
        "wetted_area_m2": sc(f, "thermal_obj/cooling/A_wetted_inner", 0.025),
        "fluid": fluid,
    }

    # Per-component loss split → reduced-group fractions (integrated over the operating grid). Iterate
    # the known component names (not a raw group) so the read stays typed.
    fracs: dict[str, float] = {}
    total = 0.0
    for comp, grp in LOSS_TO_GROUP.items():
        col = c.opt_arr(f, f"operating_grid/loss_breakdown/{comp}")
        if col is None:
            continue
        val = float(np.nansum(np.abs(col)))
        fracs[grp] = fracs.get(grp, 0.0) + val
        total += val
    if total > 0.0:
        fracs = {k: v / total for k, v in fracs.items()}
    else:
        fracs = {"winding": 1.0}

    return {
        "node_names": names,
        "c_vec": c_vec,
        "g_const": g_const,
        "air_gap_mm": air_gap_mm,
        "rotor_outer_radius_mm": r_ro_mm,
        "stack_length_mm": stack_mm,
        "jacket": jacket,
        "loss_group_fracs": fracs,
        "t_limit_winding_c": sc(f, "thermal_obj/t_limit_winding", 180.0),
        "t_limit_magnet_c": sc(f, "thermal_obj/t_limit_magnet", 150.0),
        "cu_alpha": sc(f, "thermal_obj/cu_temp_coeff", 0.00393),
        "t_ref_c": sc(f, "thermal_obj/T_ref_winding_C", 60.0),
    }


def aggregate_network(
    node_names: list[str], c_vec: np.ndarray, g_const: np.ndarray
) -> tuple[dict[str, float], list[tuple[str, str, float]]]:
    """Sum capacities per group and inter-group ``G_const`` conductances (skipping intra-group edges).

    Returns ``(capacity_by_group, [(group_a, group_b, w_per_k), …])``. Coolant/ambient capacities are
    dropped (those nodes are boundary/balance-closed).
    """
    grp = [_group_of(n) for n in node_names]
    cap: dict[str, float] = {}
    for gi, ci in zip(grp, c_vec, strict=True):
        if gi is None or gi in ("coolant", "ambient"):
            continue
        if np.isfinite(ci):
            cap[gi] = cap.get(gi, 0.0) + float(ci)
    edges: dict[tuple[str, str], float] = {}
    n = len(node_names)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = grp[i], grp[j]
            gij = float(g_const[i, j])
            if gij <= 0.0 or a is None or b is None or a == b:
                continue
            key: tuple[str, str] = (a, b) if a < b else (b, a)
            edges[key] = edges.get(key, 0.0) + gij
    edge_list = [(a, b, round(w, 4)) for (a, b), w in sorted(edges.items())]
    return cap, edge_list


def build_detailed_emotor(
    *,
    node_names: list[str],
    c_vec: np.ndarray,
    g_const: np.ndarray,
    air_gap_mm: float,
    rotor_outer_radius_mm: float,
    stack_length_mm: float,
    jacket: dict[str, Any],
    loss_group_fracs: dict[str, float],
    t_limit_winding_c: float,
    t_limit_magnet_c: float,
    cu_alpha: float | None,
    t_ref_c: float,
    provenance: str,
) -> dict[str, Any]:
    """Assemble a detailed ``emotor/1.1`` document from aggregated PDT arrays + scalar cooling fields.

    ``jacket`` carries the raw jacket settings (``inlet_c``, ``flow_rate_lps``, ``channel_count``,
    ``channel_width_mm``, ``channel_height_mm``, ``wetted_area_m2``, ``fluid``). ``loss_group_fracs``
    maps a reduced group to its fraction of the total machine loss.
    """
    cap, edges = aggregate_network(node_names, c_vec, g_const)
    present = [g for g in ("winding", "stator_iron", "rotor", "housing") if g in cap]

    limits = {
        "winding": (round(t_limit_winding_c - 25.0, 1), round(t_limit_winding_c, 1)),
        "rotor": (round(t_limit_magnet_c - 25.0, 1), round(t_limit_magnet_c, 1)),
    }
    nodes: list[dict[str, Any]] = []
    for grp in present:
        node: dict[str, Any] = {
            "name": grp,
            "role": _ROLE[grp],
            "c_j_per_k": round(cap[grp], 2),
        }
        if grp in limits:
            node["t_warn_c"], node["t_max_c"] = limits[grp]
        nodes.append(node)
    nodes.append({"name": "coolant", "role": "coolant"})
    nodes.append({"name": "ambient", "role": "ambient"})

    # Constant conduction edges among the solid groups (air-gap + jacket are convection, added below).
    conductances = [
        {"between": [a, b], "w_per_k": w}
        for (a, b, w) in edges
        if a in present and b in present and "coolant" not in (a, b)
    ]

    cooling: dict[str, Any] = {"ambient_node": "ambient"}
    cooling["jacket"] = {
        "housing_node": "housing",
        "coolant_node": "coolant",
        "inlet_c": round(float(jacket["inlet_c"]), 2),
        "flow_rate_lps": round(float(jacket["flow_rate_lps"]), 5),
        "channel_count": int(jacket["channel_count"]),
        "channel_width_mm": round(float(jacket["channel_width_mm"]), 3),
        "channel_height_mm": round(float(jacket["channel_height_mm"]), 3),
        "wetted_area_m2": round(float(jacket["wetted_area_m2"]), 5),
        "fluid": jacket["fluid"],
    }
    if "rotor" in present and "stator_iron" in present:
        cooling["air_gap"] = {
            "between": ["stator_iron", "rotor"],
            "rotor_outer_radius_mm": round(float(rotor_outer_radius_mm), 3),
            "gap_mm": round(float(air_gap_mm), 4),
            "stack_length_mm": round(float(stack_length_mm), 3),
        }

    loss_routing = [
        {"node": grp, "fraction": round(float(frac), 4)}
        for grp, frac in loss_group_fracs.items()
        if grp in present and frac > 0.0
    ]

    doc: dict[str, Any] = {
        "schema": "emotor/1.1",
        "nodes": nodes,
        "conductances": conductances,
        "cooling": cooling,
        "loss_routing": loss_routing,
        "meta": {"source": "pdt_imported", "notes": provenance},
    }
    if cu_alpha:
        doc["cu_feedback"] = {
            "nodes": ["winding"],
            "t_ref_c": round(float(t_ref_c), 2),
            "alpha_per_k": round(float(cu_alpha), 6),
        }
    return doc
