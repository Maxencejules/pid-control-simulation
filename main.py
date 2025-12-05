import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def run_simulation():
    # Time parameters
    dt = 0.1
    sim_time = 180  # seconds (3 minutes)
    steps = int(sim_time / dt)

    # System + PID
    system = ThermalSystem()  # make sure gain is 4.0 in system_model.py
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

        # Compute PID output from current measurement
        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)

        # Clamp power between 0 and 1 (like a real heater)
        heater_power = max(0.0, min(1.0, heater_power))

        # Update system with this heater power
        measurement = system.update(heater_power=heater_power, dt=dt)

        # Store logs
        times.append(current_time)
        temps.append(measurement)
        outputs.append(heater_power)
        setpoints.append(setpoint)

    # Plot results: temperature + heater power
    plt.figure(figsize=(12, 10))

    # Temperature subplot
    plt.subplot(2, 1, 1)
    plt.plot(times, temps, label="Measured Temperature")
    plt.plot(times, setpoints, "--", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("PID Temperature Control Simulation")
    plt.legend()
    plt.grid(True)

    # Heater power subplot
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
    run_simulation()
