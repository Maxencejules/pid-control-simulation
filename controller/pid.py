class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        # Error
        error = setpoint - measurement

        # Integral
        self.integral += error * self.dt

        # Derivative
        derivative = (error - self.prev_error) / self.dt

        # PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Save error
        self.prev_error = error

        return output
