"""Deterministic experiment definitions; no plotting or global RNG state."""

from dataclasses import asdict
import math

from controller.pid import PID
from simulation import Trace, response_metrics, simulate
from system.system_model import ThermalSystem
from utils.filtering import MovingAverageFilter

DT = 0.1
GAINS = (0.18, 0.015, 0.03)
SEED = 7


def step_setpoint(time):
    return 30.0 if time < 60 else 50.0 if time < 120 else 40.0


def run_experiments() -> dict[str, Trace]:
    traces = {}
    for name, gains in [("P", (0.18, 0, 0)), ("PI", (0.18, 0.015, 0)), ("PID", GAINS)]:
        traces[name] = simulate(PID(*gains, DT), 180, 50, plant=ThermalSystem(noise_std=0))
    traces["step_response"] = simulate(PID(*GAINS, DT), 180, step_setpoint,
                                       plant=ThermalSystem(noise_std=0))
    traces["ramp_limited"] = simulate(PID(*GAINS, DT, max_output_rate=0.1), 180, 50,
                                    plant=ThermalSystem(noise_std=0))
    for name, filt in [("unfiltered", None), ("filtered", MovingAverageFilter(10))]:
        traces[name] = simulate(PID(*GAINS, DT), 180, 50,
                                plant=ThermalSystem(noise_std=0.5, seed=SEED), sensor_filter=filt)
    recovery_setpoint = lambda time: 100.0 if time < 60 else 40.0
    for name, enabled in [("anti_windup", True), ("without_anti_windup", False)]:
        traces[name] = simulate(PID(*GAINS, DT, anti_windup=enabled), 240, recovery_setpoint,
                                plant=ThermalSystem(noise_std=0))
    for kp in (0.1, 5.0):
        traces[f"sampling_kp_{kp:g}"] = simulate(PID(kp, 0, 0, DT), 12, 21,
                                                 plant=ThermalSystem(noise_std=0))
    return traces


def _slice(trace, start, end, *, include_end=True):
    """Use half-open command windows; include the finite final endpoint if requested."""
    indices = [index for index, time in enumerate(trace.times)
               if start <= time and (time <= end if include_end else time < end)]
    return Trace(**{field: [values[index] for index in indices]
                    for field, values in vars(trace).items()})


def _sensor_statistics(trace):
    samples = _slice(trace, 150, 180)
    rms = lambda values: math.sqrt(sum(value*value for value in values) / len(values))
    return {
        "measurement_error_rms_last_30s_c": rms([measured-true for measured, true in
                                                 zip(samples.measurements, samples.temperatures)]),
        "feedback_error_rms_last_30s_c": rms([feedback-true for feedback, true in
                                              zip(samples.feedback, samples.temperatures)]),
        "heater_total_variation_last_30s": sum(abs(b-a) for a, b in
                                               zip(samples.powers, samples.powers[1:])),
    }


def summarize(traces):
    metrics = {name: asdict(response_metrics(traces[name], 50))
               for name in ("P", "PI", "PID", "ramp_limited", "unfiltered", "filtered")}
    for name in ("filtered", "unfiltered"):
        metrics[name] |= _sensor_statistics(traces[name])
    for name in ("anti_windup", "without_anti_windup"):
        metrics[name] = asdict(response_metrics(traces[name], 40, start=60))
        metrics[name]["peak_integral_error_seconds"] = max(traces[name].integrals)
    # Half-open intervals stop at the next command change without subtracting dt.
    metrics["step_response"] = [
        {"start_s": start, "target_c": target,
         **asdict(response_metrics(_slice(traces["step_response"], start, end,
                                         include_end=include_end), target, start=start))}
        for start, end, target, include_end in [(0, 60, 30, False), (60, 120, 50, False),
                                               (120, 180, 40, True)]
    ]
    return {
        "configuration": {
            "dt_s": DT, "ambient_c": 20, "tau_s": 12, "gain_c_per_second_per_unit_power": 4,
            "maximum_equilibrium_c": 68, "pid_gains": dict(zip(("kp", "ki", "kd"), GAINS)),
            "noise_seed": SEED, "noise_std_c": {"default": 0.0, "unfiltered": 0.5, "filtered": 0.5},
            "nominal_duration_s": 180, "recovery_duration_s": 240, "sampling_duration_s": 12,
            "recovery_release_time_s": 60, "moving_average_samples": 10,
            "ramp_limit_power_units_per_second": 0.1,
            "response_metrics_use_true_temperature": True,
            "settling_band": "2% of initial step amplitude, minimum 0.1 C",
            "steady_state_window_s": 10,
        },
        "metrics": metrics,
        "sampling_analysis": {
            "continuous_p_pole_per_second": {str(kp): -(1/12 + 4*kp) for kp in (0.1, 5.0)},
            "euler_p_pole": {str(kp): 1-DT*(1/12 + 4*kp) for kp in (0.1, 5.0)},
            "euler_linear_stability_kp_upper_bound": (2/DT - 1/12) / 4,
            "continuous_ultimate_gain": None,
        },
    }
