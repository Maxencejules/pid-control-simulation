"""A single simulation clock shared by the examples and numerical checks."""

from collections.abc import Callable
from dataclasses import dataclass
import math

from controller.pid import PID
from system.system_model import ThermalSystem
from utils.filtering import MovingAverageFilter


@dataclass
class Trace:
    times: list[float]
    temperatures: list[float]
    measurements: list[float]
    feedback: list[float]
    setpoints: list[float]
    powers: list[float]
    raw_outputs: list[float]
    integrals: list[float]


def simulate(
    pid: PID, duration: float, setpoint: float | Callable[[float], float], *,
    plant: ThermalSystem | None = None, sensor_filter: MovingAverageFilter | None = None,
) -> Trace:
    """Control at t, hold u for dt, and record the physical state at t+dt.

    powers[n] is the command applied over (times[n-1], times[n]]. Setpoints
    are sampled at the displayed boundary, so a scheduled step is visible at
    its actual time. Physical time advances exactly once per controller sample.
    """
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    steps = round(duration / pid.dt)
    if steps < 1 or not math.isclose(steps * pid.dt, duration, rel_tol=0, abs_tol=1e-9):
        raise ValueError("duration must be a whole number of controller samples")
    plant = plant if plant is not None else ThermalSystem()
    target = setpoint if callable(setpoint) else lambda _time: setpoint
    measurement = plant.measure()
    feedback = sensor_filter.filter(measurement) if sensor_filter else measurement
    trace = Trace([0.0], [plant.temperature], [measurement], [feedback],
                  [target(0.0)], [pid.output], [pid.raw_output], [pid.integral])
    for step in range(steps):
        power = pid.compute(target(step * pid.dt), feedback)
        measurement = plant.update(power, pid.dt)
        feedback = sensor_filter.filter(measurement) if sensor_filter else measurement
        time = (step + 1) * pid.dt
        trace.times.append(time)
        trace.temperatures.append(plant.temperature)
        trace.measurements.append(measurement)
        trace.feedback.append(feedback)
        trace.setpoints.append(target(time))
        trace.powers.append(power)
        trace.raw_outputs.append(pid.raw_output)
        trace.integrals.append(pid.integral)
    return trace


@dataclass(frozen=True)
class ResponseMetrics:
    overshoot_c: float
    overshoot_percent: float
    settling_time_s: float | None
    settling_band_c: float
    steady_state_error_c: float


def response_metrics(trace: Trace, target: float, *, start: float = 0.0,
                     tail_seconds: float = 10.0) -> ResponseMetrics:
    """Measure true temperature; use a 2% step band with a 0.1 C floor.

    Settling requires every remaining sample to stay in the band. Steady-state
    error is target minus mean physical temperature in the final tail window.
    A finite trace cannot establish infinite-horizon stability.
    """
    if not math.isfinite(target) or not math.isfinite(start) or start < 0:
        raise ValueError("target and start must be finite; start must be nonnegative")
    samples = [(time, temp) for time, temp in zip(trace.times, trace.temperatures)
               if time >= start]
    if len(samples) < 2 or tail_seconds <= 0 or not math.isfinite(tail_seconds):
        raise ValueError("metrics need at least two samples and a positive tail window")
    amplitude = abs(target - samples[0][1])
    direction = 1 if target >= samples[0][1] else -1
    overshoot = max(0.0, max(direction * (temp - target) for _, temp in samples))
    band = max(0.02 * amplitude, 0.1)
    last_outside = max((index for index, (_, temp) in enumerate(samples)
                        if abs(target - temp) > band), default=-1)
    settling = None if last_outside == len(samples) - 1 else (
        samples[last_outside + 1][0] - samples[0][0]
    )
    tail_start = max(samples[0][0], samples[-1][0] - tail_seconds)
    tail = [temp for time, temp in samples if time >= tail_start]
    return ResponseMetrics(overshoot, 100 * overshoot / amplitude if amplitude else 0.0,
                           settling, band, target - sum(tail) / len(tail))
