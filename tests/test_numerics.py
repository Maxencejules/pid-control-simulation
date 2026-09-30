"""Independent continuous-time oracles and physical acceptance checks."""

import math

import pytest

from controller.pid import PID
from simulation import Trace, response_metrics, simulate
from system.system_model import ThermalSystem
from utils.filtering import MovingAverageFilter
from experiments import _slice, run_experiments, summarize


def constant_power_temperature(time, power):
    # Exact ODE solution, not the plant's Euler recurrence.
    return 20.0 + 48.0 * power * (1.0 - math.exp(-time / 12.0))


def test_thermal_euler_converges_to_the_exact_constant_power_solution():
    errors = []
    for dt in (0.4, 0.2, 0.1):
        plant = ThermalSystem(noise_std=0)
        for _ in range(round(12 / dt)):
            plant.update(0.6, dt)
        errors.append(abs(plant.temperature - constant_power_temperature(12, 0.6)))
    assert 1.95 < errors[0] / errors[1] < 2.1
    assert 1.95 < errors[1] / errors[2] < 2.1
    assert errors[-1] < 0.05


def test_p_equilibrium_and_closed_loop_transient_match_the_analytical_solution():
    kp, setpoint, dt = 0.1, 21.0, 0.05
    dc_gain = 48.0
    equilibrium = 20 + dc_gain * kp * (setpoint - 20) / (1 + dc_gain * kp)
    pole = -(1 / 12 + 4 * kp)
    trace = simulate(PID(kp, 0, 0, dt), 30, setpoint, plant=ThermalSystem(noise_std=0))
    for time in (1, 5, 10, 20, 30):
        expected = equilibrium + (20 - equilibrium) * math.exp(pole * time)
        assert trace.temperatures[round(time / dt)] == pytest.approx(expected, abs=0.004)
    expected_error = (setpoint - 20) / (1 + dc_gain * kp)
    assert setpoint - trace.temperatures[-1] == pytest.approx(expected_error, abs=1e-6)
    assert all(0 < power < 1 for power in trace.powers[1:])


def continuous_pi_temperature(time):
    # Solve y'' + a*y' + b*y = b*R, y(0)=0, y'(0)=gain*kp*R.
    kp, ki, gain, tau, reference = 0.1, 0.008, 4.0, 12.0, 1.0
    a, b = 1 / tau + gain * kp, gain * ki
    root1 = (-a + math.sqrt(a*a - 4*b)) / 2
    root2 = (-a - math.sqrt(a*a - 4*b)) / 2
    coefficient1 = (gain * kp * reference + reference * root2) / (root1 - root2)
    coefficient2 = -reference - coefficient1
    return 20 + reference + coefficient1 * math.exp(root1*time) + coefficient2 * math.exp(root2*time)


def test_unsaturated_pi_converges_to_independent_continuous_reference():
    errors = []
    for dt in (0.2, 0.1, 0.05):
        trace = simulate(PID(0.1, 0.008, 0, dt), 20, 21, plant=ThermalSystem(noise_std=0))
        # The oracle assumes no actuator saturation; verify that precondition.
        assert all(0 < power < 1 for power in trace.powers[1:])
        errors.append(abs(trace.temperatures[-1] - continuous_pi_temperature(20)))
    assert 1.9 < errors[0] / errors[1] < 2.1
    assert 1.9 < errors[1] / errors[2] < 2.1
    assert errors[-1] < 0.00025


@pytest.mark.parametrize("dt", [0.2, 0.1, 0.05])
def test_nominal_pid_converges_and_respects_physical_bounds(dt):
    trace = simulate(PID(0.18, 0.015, 0.03, dt), 120, 50, plant=ThermalSystem(noise_std=0))
    metrics = response_metrics(trace, 50)
    assert all(0 <= power <= 1 for power in trace.powers)
    assert all(20 <= temperature <= 68 for temperature in trace.temperatures)
    assert metrics.overshoot_c < 0.1
    assert metrics.settling_time_s is not None and metrics.settling_time_s < 32
    assert abs(metrics.steady_state_error_c) < 0.001
    assert trace.powers[-1] == pytest.approx((50 - 20) / 48, abs=0.001)


def test_unreachable_demand_release_demonstrates_anti_windup_recovery():
    def target(time):
        return 100.0 if time < 60 else 40.0
    traces = [simulate(PID(0.18, 0.015, 0.03, 0.1, anti_windup=enabled), 240, target,
                       plant=ThermalSystem(noise_std=0)) for enabled in (True, False)]
    protected, unprotected = traces
    for trace in traces:
        assert all(0 <= power <= 1 for power in trace.powers)
        assert max(trace.temperatures) <= 68
    good, bad = [response_metrics(trace, 40, start=60) for trace in traces]
    assert good.settling_time_s is not None and good.settling_time_s < 35
    assert bad.settling_time_s is not None and bad.settling_time_s > 100
    assert protected.temperatures[1200] == pytest.approx(40, abs=0.1)
    assert unprotected.temperatures[1200] > 67
    assert max(protected.integrals[:601]) == 0.0
    assert max(unprotected.integrals[:601]) > 2000


def test_pure_integral_heater_reaches_full_power_equilibrium_and_releases():
    target = lambda time: 68.0 if time < 120 else 40.0
    trace = simulate(PID(0, 0.1, 0, 0.1), 300, target, plant=ThermalSystem(noise_std=0))
    assert trace.powers[3] == 1.0
    assert trace.temperatures[1200] == pytest.approx(68, abs=0.01)
    assert min(trace.powers[1201:1300]) == 0.0
    assert abs(response_metrics(trace, 40, start=120).steady_state_error_c) < 0.02


def test_sensor_noise_is_reproducible_and_never_changes_the_plant_state():
    quiet, noisy = ThermalSystem(noise_std=0), ThermalSystem(noise_std=0.5, seed=7)
    repeated = ThermalSystem(noise_std=0.5, seed=7)
    noise_seen = False
    for _ in range(100):
        clean_measurement = quiet.update(0.5, 0.1)
        noisy_measurement = noisy.update(0.5, 0.1)
        assert repeated.update(0.5, 0.1) == noisy_measurement
        assert quiet.temperature == noisy.temperature == repeated.temperature
        noise_seen |= noisy_measurement != clean_measurement
    assert noise_seen
    before = noisy.temperature
    noisy.measure()
    assert noisy.temperature == before


def test_filtered_simulation_uses_one_physical_update_per_sample_and_correct_clock():
    dt, duration = 0.1, 1.0
    trace = simulate(PID(1, 0, 0, dt), duration, 1000,
                     plant=ThermalSystem(noise_std=0), sensor_filter=MovingAverageFilter(5))
    assert trace.times == pytest.approx([index * dt for index in range(11)])
    # Ten full-power Euler updates; a second update per sample breaks this value.
    expected = 20 + 48 * (1 - (1 - dt / 12) ** 10)
    assert trace.temperatures[-1] == pytest.approx(expected)
    assert len(trace.measurements) == len(trace.feedback) == 11


def test_seeded_filter_comparison_uses_matched_noise_and_reduces_feedback_noise_and_chatter():
    traces = run_experiments()
    raw, filtered = traces["unfiltered"], traces["filtered"]
    raw_errors = [measurement-true for measurement, true in zip(raw.measurements, raw.temperatures)]
    filtered_errors = [measurement-true for measurement, true in zip(filtered.measurements, filtered.temperatures)]
    assert filtered_errors == pytest.approx(raw_errors, abs=1e-12)
    report = summarize(traces)["metrics"]
    assert report["filtered"]["feedback_error_rms_last_30s_c"] < 0.5 * report["unfiltered"]["feedback_error_rms_last_30s_c"]
    assert report["filtered"]["heater_total_variation_last_30s"] < 0.15 * report["unfiltered"]["heater_total_variation_last_30s"]
    assert abs(report["filtered"]["steady_state_error_c"]) < 0.2
    assert summarize(run_experiments()) == summarize(traces)


def test_rate_limited_closed_loop_settles_and_obeys_the_limit_throughout():
    trace = simulate(PID(.18, .015, .03, .1, max_output_rate=.1), 180, 50,
                     plant=ThermalSystem(noise_std=0))
    assert all(abs(b-a) <= .01 + 1e-12 for a, b in zip(trace.powers, trace.powers[1:]))
    metrics = response_metrics(trace, 50)
    assert metrics.settling_time_s is not None and metrics.settling_time_s < 40
    assert abs(metrics.steady_state_error_c) < 0.001


def test_step_metrics_use_direction_and_require_remaining_samples_in_band():
    trace = Trace([0, 1, 2, 3, 4], [50, 39, 40, 41, 40], [], [], [], [], [], [])
    metrics = response_metrics(trace, 40, tail_seconds=1)
    assert metrics.overshoot_c == 1.0
    assert metrics.overshoot_percent == 10.0
    assert metrics.settling_band_c == 0.2
    assert metrics.settling_time_s == 4.0  # The first crossing at t=2 was temporary.
    assert metrics.steady_state_error_c == -0.5
    unfinished = Trace([0, 1], [20, 22], [], [], [], [], [], [])
    assert response_metrics(unfinished, 50).settling_time_s is None


def test_step_segment_boundaries_keep_sample_599_and_exclude_the_next_command():
    traces = run_experiments()
    trace = traces["step_response"]
    assert trace.times[599] > 59.9  # Binary float rounding must not drop this sample.
    expected_tail_error = 30 - sum(trace.temperatures[499:600]) / 101
    report = summarize(traces)["metrics"]["step_response"]
    assert report[0]["steady_state_error_c"] == pytest.approx(expected_tail_error, abs=1e-12)
    first = _slice(trace, 0, 60, include_end=False)
    second = _slice(trace, 60, 120, include_end=False)
    final = _slice(trace, 120, 180)
    assert first.times == trace.times[:600]
    assert first.times[-1] == trace.times[599]
    assert 60 not in first.times
    assert second.times == trace.times[600:1200]
    assert final.times == trace.times[1200:]
    assert final.times[-1] == 180


def test_moving_average_warmup_has_no_zero_padding():
    filt = MovingAverageFilter(3)
    assert [filt.filter(value) for value in [1, 3, 5, 7]] == [1, 2, 3, 5]
    with pytest.raises(ValueError):
        MovingAverageFilter(0)


@pytest.mark.parametrize("power,dt", [(-0.1, 0.1), (1.1, 0.1), (0.5, 0), (0.5, 13)])
def test_invalid_thermal_actuation_and_timestep_are_rejected(power, dt):
    with pytest.raises(ValueError):
        ThermalSystem().update(power, dt)
