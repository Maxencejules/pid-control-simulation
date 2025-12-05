import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def run_ramp_limited_control():
    dt = 0.1
    sim_time = 180.0
    steps = int(sim_time / dt)

    system = ThermalSystem()
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)
    setpoint = 50.0

    max_delta = 0.05  # max change per step
    heater_power = 0.0

    times, temps, outputs = [], [], []
    measurement = system.temperature

    for i in range(steps):
        t = i * dt

        desired = pid.compute(setpoint=setpoint, measurement=measurement)

        desired = max(0.0, min(1.0, desired))

        # Ramp-rate limiting
        delta = desired - heater_power
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta

        heater_power += delta

        measurement = system.update(heater_power, dt)

        times.append(t)
        temps.append(measurement)
        outputs.append(heater_power)

    plt.figure(figsize=(12, 10))

    plt.subplot(2, 1, 1)
    plt.plot(times, temps, label="Measured Temperature")
    plt.axhline(setpoint, linestyle="--", color="orange", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("PID Control with Ramp-Limited Heater Output")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(times, outputs, label="Heater Power (ramp-limited)")
    plt.xlabel("Time (s)")
    plt.ylabel("Power")
    plt.title("Heater Output With Ramp Rate Limiting")
    plt.ylim(0, 1.05)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_ramp_limited_control()
