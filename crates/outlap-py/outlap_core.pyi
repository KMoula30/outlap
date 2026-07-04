# SPDX-License-Identifier: AGPL-3.0-only
"""Type stubs for the `outlap_core` extension module (shipped in the wheel by maturin)."""

import numpy as np
from numpy.typing import NDArray

DEFAULT_DS_M: float

class Tyre:
    notes: list[tuple[str, str]]
    citation: str
    fnomin: float
    unloaded_radius: float
    p_cold: float
    @staticmethod
    def load(path: str) -> Tyre: ...
    def forces(
        self,
        kappa: NDArray[np.float64],
        alpha: NDArray[np.float64],
        gamma: NDArray[np.float64],
        fz: NDArray[np.float64],
        p: NDArray[np.float64],
        vx: NDArray[np.float64],
    ) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]: ...
    def peak_mu(self, fz: float, p: float) -> tuple[float, float]: ...

class Track:
    @staticmethod
    def load(dir: str) -> Track: ...
    def name(self) -> str: ...
    def length(self) -> float: ...
    def is_closed(self) -> bool: ...
    def sample(self, ds_m: float) -> dict[str, NDArray[np.float64]]: ...

class Raceline:
    ds_m: float
    def s(self) -> NDArray[np.float64]: ...
    def n(self) -> NDArray[np.float64]: ...
    def line(self) -> Track: ...

class Lap:
    lap_time_s: float
    notes: list[str]
    resolved_hash: str
    def s(self) -> NDArray[np.float64]: ...
    def v(self) -> NDArray[np.float64]: ...
    def ax(self) -> NDArray[np.float64]: ...
    def ay(self) -> NDArray[np.float64]: ...
    def t(self) -> NDArray[np.float64]: ...
    def x(self) -> NDArray[np.float64]: ...
    def y(self) -> NDArray[np.float64]: ...
    def z(self) -> NDArray[np.float64]: ...

def min_curvature(
    track: Track,
    half_width_m: float,
    ds_m: float = 2.0,
    margin_m: float = 0.3,
    epsilon: float = 1e-8,
) -> Raceline: ...
def solve_lap(
    vehicle_dir: str,
    track: Track,
    ds_m: float = ...,
    raceline_ds_m: float | None = None,
) -> Lap: ...
def vehicle_report(vehicle_dir: str) -> dict[str, object]: ...
