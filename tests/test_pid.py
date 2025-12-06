from controller.pid import PID

def test_zero_error_gives_zero_output():
    pid = PID(kp=2.0, ki=1.0, kd=0.5, dt=0.1)
    out = pid.compute(setpoint=10.0, measurement=10.0)
    assert abs(out) < 1e-6

def test_positive_error_gives_positive_output():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, dt=0.1)
    out = pid.compute(setpoint=20.0, measurement=10.0)
    assert out > 0.0
