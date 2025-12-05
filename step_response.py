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

    plt.figure(figsize=(12, 10))

    plt.subplot(2, 1, 1)
    plt.plot(times, temps, label="Measured Temperature")
    plt.plot(times, setpoints, "--", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("PID Step Response: 30°C → 50°C → 40°C")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(times, outputs, label="Heater Power (0–1)")
    plt.xlabel("Time (s)")
    plt.ylabel("Power")
    plt.title("Heater Output Over Time")
    plt.ylim(0, 1.05)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_step_response()
