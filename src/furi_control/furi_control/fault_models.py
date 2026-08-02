from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class FrictionConfig:
    viscous: FloatArray
    coulomb: FloatArray
    smoothing_velocity: float


def apply_friction_proxy(
    velocity: FloatArray,
    config: FrictionConfig,
) -> FloatArray:
    opposing = (
        config.viscous * velocity
        + config.coulomb
        * np.tanh(velocity / config.smoothing_velocity)
    )
    return velocity - opposing


def add_gaussian_noise(
    values: FloatArray,
    standard_deviation: FloatArray,
    rng: np.random.Generator,
) -> FloatArray:
    return values + rng.normal(
        loc=0.0,
        scale=standard_deviation,
        size=values.shape,
    )