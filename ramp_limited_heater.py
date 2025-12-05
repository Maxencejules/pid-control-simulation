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

    max_delta = 0.05  # max allowed change per step (ramp limit)
    heater_power = 0.0

    times, temps, outputs = [], [], []
    measurement = system.temperature

    for i in range(steps):
        t = i * dt

        # Ideal PID output
        desired = pid.compute(setpoint=setpoint, measurement=measurement)
        desired = max(0.0, min(1.0, desired))

        # Apply ramp-rate limit
        delta = desired - heater_power
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta

        heater_power += delta

        # Update system
        measurement = system.update(heater_power, dt)

        # Log
        times.append(t)
        temps.append(measurement)
        outputs.append(heater_power)

    # --- Clean subplot layout ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Temperature subplot
    ax1 = axes[0]
    ax1.plot(times, temps, label="Measured Temperature")
    ax1.axhline(setpoint, linestyle="--", color="orange", label="Setpoint")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("PID Control with Ramp-Limited Heater Output")
    ax1.legend()
    ax1.grid(True)

    # Heater power subplot
    ax2 = axes[1]
    ax2.plot(times, outputs, label="Heater Power (Ramp-Limited)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Power")
    ax2.set_title("Heater Output With Ramp Rate Limiting")
    ax2.set_ylim(0, 1.05)
    ax2.legend()
    ax2.grid(True)

    # Layout fix
    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.35)

    plt.show()


if __name__ == "__main__":
    run_ramp_limited_control()
