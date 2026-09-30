from controller.pid import PID
import pytest

def test_zero_error_gives_zero_output():
    pid = PID(kp=2.0, ki=1.0, kd=0.5, dt=0.1)
    out = pid.compute(setpoint=10.0, measurement=10.0)
    assert abs(out) < 1e-6

def test_positive_error_gives_positive_output():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, dt=0.1)
    out = pid.compute(setpoint=20.0, measurement=10.0)
    assert out > 0.0


def test_controller_returns_the_applied_saturated_power():
    pid = PID(2.0, 1.0, 0.0, 0.1)
    assert pid.compute(50.0, 20.0) == 1.0
    assert pid.compute(10.0, 20.0) == 0.0
    assert pid.integral == 0.0


@pytest.mark.parametrize("integral,error,expected", [(10.0, -1.0, 9.0), (-10.0, 1.0, -9.0)])
def test_an_error_toward_the_valid_range_unwinds_even_while_saturated(integral, error, expected):
    pid = PID(0.0, 1.0, 0.0, 1.0)
    pid.integral = integral  # A stored state from an earlier unprotected period.
    pid.compute(error, 0.0)
    assert pid.integral == expected


def test_derivative_on_measurement_does_not_kick_on_a_setpoint_step():
    pid = PID(0.0, 0.0, 1.0, 0.1)
    assert pid.compute(20.0, 20.0) == 0.0
    assert pid.compute(50.0, 20.0) == 0.0
    assert pid.compute(50.0, 19.0) == 1.0
    pid.reset()
    assert pid.compute(50.0, 19.0) == 0.0
    assert pid.integral == 0.0


def test_slew_limit_is_in_power_units_per_second_and_blocks_windup():
    pid = PID(0.2, 0.1, 0.0, 0.1, max_output_rate=0.05)
    powers = [0.0] + [pid.compute(50.0, 20.0) for _ in range(20)]
    assert powers[-1] == pytest.approx(0.1)
    assert all(abs(b-a) <= 0.005 + 1e-12 for a, b in zip(powers, powers[1:]))
    assert pid.integral == 0.0
    assert pid.compute(0.0, 20.0) == pytest.approx(0.095)
    assert pid.integral == 0.0


def test_pure_integral_controller_can_start_and_cross_an_actuator_boundary():
    pid = PID(0.0, 1.0, 0.0, 0.1)
    assert pid.compute(50.0, 20.0) == 1.0  # Proposed integral output is 3, not 0.
    assert pid.integral == 3.0
    pid.compute(50.0, 20.0)
    assert pid.integral == 3.0  # Further outward accumulation stops.
    pid.compute(0.0, 20.0)
    assert pid.integral == 1.0  # Inward error still releases the stored integral.
    gentle = PID(0.0, 1.0, 0.0, 0.1)
    outputs = [gentle.compute(1.0, 0.0) for _ in range(20)]
    assert outputs[-1] == 1.0


@pytest.mark.parametrize("kwargs", [
    {"dt": 0.0}, {"dt": float("nan")}, {"kp": -1.0}, {"ki": float("inf")},
    {"output_limits": (1.0, 0.0)}, {"max_output_rate": 0.0},
])
def test_invalid_controller_configuration_is_rejected(kwargs):
    settings = {"kp": 0.1, "ki": 0.01, "kd": 0.0, "dt": 0.1} | kwargs
    with pytest.raises(ValueError):
        PID(**settings)


def test_nonfinite_sensor_input_is_rejected():
    with pytest.raises(ValueError):
        PID(0.1, 0.01, 0.0, 0.1).compute(50, float("nan"))
