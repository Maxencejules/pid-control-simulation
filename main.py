import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def run_simulation():
    # Time parameters
    dt = 0.1
    sim_time = 180  # seconds (3 minutes)
    steps = int(sim_time / dt)

    # System + PID
    system = ThermalSystem()  # uses gain=4.0, tau=12.0
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)

    # Target temp
    setpoint = 50.0

    # Logging
    times = []
    temps = []
    outputs = []
    setpoints = []

    # Start from current system temp
    measurement = system.temperature

    for i in range(steps):
        current_time = i * dt

        # PID control
        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)
        heater_power = max(0.0, min(1.0, heater_power))  # clamp 0–1

        # Update plant
        measurement = system.update(heater_power=heater_power, dt=dt)

        # Log
        times.append(current_time)
        temps.append(measurement)
        outputs.append(heater_power)
        setpoints.append(setpoint)

    # Plots using a shared figure and axes
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Temperature subplot
    ax1 = axes[0]
    ax1.plot(times, temps, label="Measured Temperature")
    ax1.plot(times, setpoints, "--", label="Setpoint")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("PID Temperature Control Simulation")
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

    # Layout fixes so titles do not jump
    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.35)

    plt.show()


if __name__ == "__main__":
    run_simulation()
