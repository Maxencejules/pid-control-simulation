import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def step_setpoint(t: float) -> float:
    if t < 60.0:
        return 30.0
    elif t < 120.0:
        return 50.0
    else:
        return 40.0


def run_step_response():
    dt = 0.1
    sim_time = 180.0
    steps = int(sim_time / dt)

    system = ThermalSystem()
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)

    times, temps, outputs, setpoints = [], [], [], []

    measurement = system.temperature

    for i in range(steps):
        t = i * dt
        setpoint = step_setpoint(t)

        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)
        heater_power = max(0.0, min(1.0, heater_power))

        measurement = system.update(heater_power, dt)

        times.append(t)
        temps.append(measurement)
        outputs.append(heater_power)
        setpoints.append(setpoint)

    # CLEAN subplot template (same as main.py version)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Temperature subplot
    ax1 = axes[0]
    ax1.plot(times, temps, label="Measured Temperature")
    ax1.plot(times, setpoints, "--", label="Setpoint")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("PID Step Response: 30°C → 50°C → 40°C")
    ax1.legend()
    ax1.grid(True)

    # Heater power subplot
    ax2 = axes[1]
    ax2.plot(times, outputs, label="Heater Power (0–1)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Power")
    ax2.set_title("Heater Output Over Time")
    ax2.set_ylim(0, 1.05)
    ax2.legend()
    ax2.grid(True)

    # Layout fixes to prevent title jumping / overlap
    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.35)

    plt.show()


if __name__ == "__main__":
    run_step_response()
