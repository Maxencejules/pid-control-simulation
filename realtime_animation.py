"""Replay a precomputed deterministic trace; redraws never advance the plant."""
from demo import plotting
from controller.pid import PID
from experiments import DT, GAINS, SEED
from simulation import simulate
from system.system_model import ThermalSystem


def main():
    plt = plotting(show=True)
    from matplotlib.animation import FuncAnimation
    trace = simulate(PID(*GAINS, DT), 60, 50, plant=ThermalSystem(seed=SEED))
    fig, (temperature_axis, power_axis) = plt.subplots(2, 1, figsize=(10, 6), layout="constrained")
    temperature_line, = temperature_axis.plot([], [], label="True temperature")
    sensor_line, = temperature_axis.plot([], [], alpha=.4, label="Sensor measurement")
    power_line, = power_axis.plot([], [], drawstyle="steps-pre", label="Applied power")
    temperature_axis.axhline(50, color="black", linestyle="--", label="Setpoint")
    temperature_axis.set(xlim=(0, 60), ylim=(19, 53), ylabel="Temperature (°C)",
                         title="Deterministic 10Hz simulation replay")
    power_axis.set(xlim=(0, 60), ylim=(-.05, 1.05), ylabel="Power (0–1)", xlabel="Time (s)")
    for axis in (temperature_axis, power_axis):
        axis.grid(True, alpha=.25)
        axis.legend()

    def init():
        temperature_line.set_data([], [])
        sensor_line.set_data([], [])
        power_line.set_data([], [])
        return temperature_line, sensor_line, power_line

    def update(frame):
        end = frame + 1
        temperature_line.set_data(trace.times[:end], trace.temperatures[:end])
        sensor_line.set_data(trace.times[:end], trace.measurements[:end])
        power_line.set_data(trace.times[:end], trace.powers[:end])
        return temperature_line, sensor_line, power_line

    animation = FuncAnimation(fig, update, init_func=init, frames=range(1, len(trace.times)),
                              interval=DT*1000, repeat=False)
    plt.show()
    return animation


if __name__ == "__main__":
    main()
