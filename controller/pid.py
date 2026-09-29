"""Sampled PID with a bounded heater and conditional-integration anti-windup."""

import math


class PID:
    def __init__(
        self, kp: float, ki: float, kd: float, dt: float, *,
        output_limits: tuple[float, float] = (0.0, 1.0),
        max_output_rate: float | None = None, anti_windup: bool = True,
    ):
        if not all(math.isfinite(value) and value >= 0 for value in (kp, ki, kd)):
            raise ValueError("PID gains must be finite and nonnegative")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        lower, upper = output_limits
        if not all(math.isfinite(value) for value in output_limits) or lower >= upper:
            raise ValueError("output limits must be finite and increasing")
        if max_output_rate is not None and (
            not math.isfinite(max_output_rate) or max_output_rate <= 0
        ):
            raise ValueError("max_output_rate must be finite and positive")
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.output_limits = output_limits
        self.max_output_rate = max_output_rate
        self.anti_windup = anti_windup
        self.reset()

    def reset(self):
        """Clear controller history; start the actuator at its lower limit."""
        self.integral = 0.0  # error-seconds, not an actuator-power value
        self.prev_error = 0.0
        self._previous_measurement = None
        self.raw_output = 0.0
        self.output = self.output_limits[0]

    def _limit(self, requested: float) -> float:
        lower, upper = self.output_limits
        bounded = max(lower, min(upper, requested))
        if self.max_output_rate is not None:
            change = self.max_output_rate * self.dt
            bounded = max(self.output - change, min(self.output + change, bounded))
        return bounded

    def compute(self, setpoint: float, measurement: float) -> float:
        """Return the applied command, including magnitude and optional slew limits.

        Derivative is taken on measurement to avoid a setpoint derivative kick.
        Integrate only when the current command can be applied, or when the
        integral increment moves the requested command toward the applied one.
        The same condition handles both magnitude saturation and slew limiting.
        """
        if not all(math.isfinite(value) for value in (setpoint, measurement)):
            raise ValueError("setpoint and measurement must be finite")
        error = setpoint - measurement
        derivative = 0.0 if self._previous_measurement is None else (
            -(measurement - self._previous_measurement) / self.dt
        )
        base = self.kp * error + self.kd * derivative
        increment = error * self.dt if self.ki else 0.0
        requested = base + self.ki * self.integral
        applied = self._limit(requested)
        pushes_further_into_limit = self.ki * increment * (requested - applied) > 0
        if not self.anti_windup or not pushes_further_into_limit:
            self.integral += increment
        self.raw_output = base + self.ki * self.integral
        self.output = self._limit(self.raw_output)
        self.prev_error = error
        self._previous_measurement = measurement
        return self.output
