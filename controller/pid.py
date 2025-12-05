class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

        # Anti-windup clamp limits (tuneable)
        self.integral_min = -10.0
        self.integral_max = 10.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        # Calculate error
        error = setpoint - measurement

        # Derivative term
        derivative = (error - self.prev_error) / self.dt

        # --- Anti-Windup: conditional integration ---
        # Only integrate if the output is not saturated
        # or if the error pushes output back toward valid range.
        raw_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        if 0 < raw_output < 1:
            self.integral += error * self.dt

            # Clamp integral term to prevent runaway
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))

        # Recalculate output with updated integral
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Save error
        self.prev_error = error

        return output
