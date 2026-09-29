"""First-order thermal plant with Euler integration and separate sensor noise."""

import math
import random


class ThermalSystem:
    """dT/dt = -(T - ambient)/tau + gain*u.

    T and ambient are Celsius, tau and dt are seconds, gain is Celsius/second
    at full heater power, and u is dimensionless in [0, 1]. Noise affects only
    measurements, never the physical temperature state. Each plant owns its RNG.
    """

    def __init__(self, ambient_temp=20.0, tau=12.0, gain=4.0, noise_std=0.2, *, seed=0):
        if not math.isfinite(ambient_temp):
            raise ValueError("ambient temperature must be finite")
        if not math.isfinite(tau) or tau <= 0:
            raise ValueError("tau must be finite and positive")
        if not all(math.isfinite(value) and value >= 0 for value in (gain, noise_std)):
            raise ValueError("gain and noise_std must be finite and nonnegative")
        self.temperature = ambient_temp
        self.ambient, self.tau, self.gain, self.noise_std = ambient_temp, tau, gain, noise_std
        self._random = random.Random(seed)

    def measure(self) -> float:
        """Read the sensor without advancing physical time."""
        return self.temperature + self._random.gauss(0.0, self.noise_std)

    def update(self, heater_power: float, dt: float) -> float:
        """Advance once, then measure; dt <= tau keeps Euler updates monotone."""
        if not math.isfinite(heater_power) or not 0 <= heater_power <= 1:
            raise ValueError("heater_power must be in [0, 1]")
        if not math.isfinite(dt) or not 0 < dt <= self.tau:
            raise ValueError("dt must be positive and no larger than tau")
        self.temperature += dt * (
            -(self.temperature - self.ambient) / self.tau + self.gain * heater_power
        )
        return self.measure()
