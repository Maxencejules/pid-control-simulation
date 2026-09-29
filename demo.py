"""Export reproducible plots, numerical summaries, and complete trace CSV files."""

import argparse
import csv
import json
import os
from pathlib import Path

from experiments import run_experiments, summarize

PLOTS = ("baseline_pid", "step_response", "filtered_control", "compare_pid_modes",
         "ramp_limited_heater", "anti_windup_recovery", "sampling_limit")


def plotting(show=False):
    # Keep Matplotlib's font cache out of the user's home directory.
    os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).parent / "build" / "mpl-cache"))
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "svg.hashsalt": "pid-control-simulation"})
    return plt


def make_figure(name, traces, plt):
    if name == "compare_pid_modes":
        fig, axis = plt.subplots(figsize=(10, 5), layout="constrained")
        for mode in ("P", "PI", "PID"):
            axis.plot(traces[mode].times, traces[mode].temperatures, label=mode)
        axis.axhline(50, color="black", linestyle="--", label="Setpoint")
        axis.set(title="P / PI / PID: identical noiseless plant", xlabel="Time (s)", ylabel="True temperature (°C)")
        axes = [axis]
    elif name == "sampling_limit":
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), layout="constrained")
        axis = axes[0]
        for kp in (0.1, 5.0):
            trace = traces[f"sampling_kp_{kp:g}"]
            axis.plot(trace.times, trace.temperatures, label=f"Kp={kp:g}, Euler pole={1-.1*(1/12+4*kp):.4f}")
        axis.axhline(21, color="black", linestyle="--", label="Setpoint")
        axis.set(title="Sampled P control: a time-step limit, not a continuous ultimate gain",
                 xlabel="Time (s)", ylabel="True temperature (°C)")
        fast = traces["sampling_kp_5"]
        axes[1].plot(fast.times, fast.temperatures, marker=".", label="Kp=5 after clipping")
        axes[1].axhline(21, color="black", linestyle="--", label="Setpoint")
        late = [temperature for time, temperature in zip(fast.times, fast.temperatures) if time >= 10]
        axes[1].set(xlim=(10, 12), ylim=(min(late)-.001, max(21, max(late))+.001),
                    xlabel="Time (s), late-response detail", ylabel="True temperature (°C)")
    elif name == "anti_windup_recovery":
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
        for key, label in [("anti_windup", "Conditional integration"), ("without_anti_windup", "No anti-windup")]:
            trace = traces[key]
            axes[0].plot(trace.times, trace.temperatures, label=label)
            axes[1].step(trace.times, trace.powers, where="pre", label=label)
            axes[2].plot(trace.times, trace.integrals, label=label)
        trace = traces["anti_windup"]
        axes[0].step(trace.times, trace.setpoints, where="post", color="black", linestyle="--", label="Setpoint")
        axes[0].axhline(68, linestyle=":", color="gray", label="Full-power equilibrium (68°C)")
        axes[0].set(title="Unreachable 100°C demand released to 40°C at t=60s", ylabel="True temperature (°C)")
        axes[1].set(ylabel="Applied power (0–1)", ylim=(-0.05, 1.05))
        axes[2].set(ylabel="Integral (°C·s)", xlabel="Time (s)")
    elif name == "filtered_control":
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), layout="constrained")
        trace = traces["filtered"]
        axes[0].plot(trace.times, trace.measurements, alpha=.35, label="Raw sensor")
        axes[0].plot(trace.times, trace.feedback, label="10-sample moving average")
        axes[0].plot(trace.times, trace.temperatures, label="True state")
        axes[0].axhline(50, color="black", linestyle="--", label="Setpoint")
        axes[0].set(title="Seeded 0.5°C measurement noise; one plant update per sample", xlabel="Time (s)", ylabel="Temperature (°C)")
        axes[1].plot(trace.times, trace.measurements, alpha=.4, label="Raw sensor")
        axes[1].plot(trace.times, trace.feedback, label="Filtered feedback")
        axes[1].plot(trace.times, trace.temperatures, label="True state")
        late = [temperature for time, temperature in zip(trace.times, trace.measurements) if time >= 150]
        axes[1].set(xlim=(150, 180), ylim=(min(late)-.1, max(late)+.1),
                    xlabel="Time (s), steady-state detail", ylabel="Temperature (°C)")
        for key in ("unfiltered", "filtered"):
            trace = traces[key]
            axes[2].step(trace.times, trace.powers, where="pre", label=key.capitalize())
        axes[2].set(xlim=(150, 180), xlabel="Time (s)", ylabel="Applied power (0–1)", ylim=(-.05, 1.05))
    else:
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, layout="constrained")
        key = {"baseline_pid": "PID", "step_response": "step_response",
               "ramp_limited_heater": "ramp_limited"}[name]
        trace = traces[key]
        axes[0].plot(trace.times, trace.temperatures, label="True temperature")
        axes[0].step(trace.times, trace.setpoints, where="post", linestyle="--", color="black", label="Setpoint")
        axes[1].step(trace.times, trace.powers, where="pre", label="Applied power")
        if name == "ramp_limited_heater":
            baseline = traces["PID"]
            axes[0].plot(baseline.times, baseline.temperatures, linestyle=":", label="Without slew limit")
            axes[1].step(baseline.times, baseline.powers, where="pre", linestyle=":", label="Without slew limit")
        axes[0].set(title={"baseline_pid": "PID: 20°C → 50°C (noiseless reference)",
                           "step_response": "PID setpoint schedule: 30°C → 50°C → 40°C",
                           "ramp_limited_heater": "Actuator slew limited to 0.1 power units/s"}[name],
                    ylabel="True temperature (°C)")
        axes[1].set(ylabel="Applied power (0–1)", xlabel="Time (s)", ylim=(-0.05, 1.05))
    for axis in axes:
        axis.grid(True, alpha=.25)
        axis.legend(loc="best", fontsize=8)
        axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    return fig


def show_experiment(name):
    plt = plotting(show=True)
    traces = run_experiments()
    make_figure(name, traces, plt)
    print(json.dumps(summarize(traces)["metrics"], indent=2))
    plt.show()
    plt.close("all")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("."))
    parser.add_argument("--show", action="store_true", help="Also display figures interactively")
    parser.add_argument("--experiment", choices=("all", *PLOTS), default="all")
    args = parser.parse_args(argv)
    plt = plotting(args.show)
    traces = run_experiments()
    image_dir, results_dir = args.output_dir / "images", args.output_dir / "results"
    image_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)
    names = PLOTS if args.experiment == "all" else (args.experiment,)
    for name in names:
        figure = make_figure(name, traces, plt)
        figure.savefig(image_dir / f"{name}.png", dpi=150, metadata={"Software": "pid-control-simulation"})
        figure.savefig(image_dir / f"{name}.svg", metadata={"Date": None, "Creator": "pid-control-simulation"})
        if not args.show:
            plt.close(figure)
    report = summarize(traces)
    (results_dir / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for name, trace in traces.items():
        with (results_dir / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "true_temperature_c", "sensor_temperature_c", "feedback_temperature_c",
                             "setpoint_c", "applied_power", "requested_power", "integral_c_seconds"])
            writer.writerows([[f"{value:.10f}" for value in row] for row in zip(*vars(trace).values())])
    print(json.dumps(report["metrics"], indent=2))
    print(f"Saved {len(names)} PNG/SVG figures, metrics.json and {len(traces)} trace CSVs to {args.output_dir.resolve()}")
    if args.show:
        plt.show()
        plt.close("all")


if __name__ == "__main__":
    main()
