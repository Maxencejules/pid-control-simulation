import random

class ThermalSystem:
    """
    Simple first-order thermal system:
    dT/dt = -(T - ambient)/tau + k * heater_power
    """

    def __init__(self, ambient_temp=20.0, tau=12.0, gain = 4.0, noise_std=0.2):
        self.temperature = ambient_temp
        self.ambient = ambient_temp
        self.tau = tau
        self.gain = gain
        self.noise_std = noise_std

    def update(self, heater_power: float, dt: float) -> float:
        # Add noise to simulate imperfect measurement (ADC noise)
        noise = random.gauss(0, self.noise_std)

        # Basic physics update
        dTdt = -(self.temperature - self.ambient) / self.tau + self.gain * heater_power
        self.temperature += dTdt * dt

        return self.temperature + noise
