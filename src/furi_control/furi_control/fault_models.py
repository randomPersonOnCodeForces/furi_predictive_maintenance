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
    # This is command attenuation, not a physical friction torque. A large
    # proxy must stop a joint rather than reverse its requested direction.
    return np.sign(velocity) * np.maximum(0.0, np.abs(velocity) - np.abs(opposing))


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
