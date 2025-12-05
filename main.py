import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def run_simulation():
    # Time parameters
    dt = 0.1
    sim_time = 180  # seconds (3 minutes)
    steps = int(sim_time / dt)

    # System + PID
    system = ThermalSystem()
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)

    # Target temp
    setpoint = 50.0

    # Logging
    times = []
    temps = []
    outputs = []
    setpoints = []

    for i in range(steps):
        current_time = i * dt

        # Read current temp
        measurement = system.update(heater_power=0, dt=dt)

        # Compute PID output
        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)

        # Clamp power between 0 and 1 (like a real heater)
        heater_power = max(0.0, min(1.0, heater_power))

        # Apply heater power
        measurement = system.update(heater_power, dt)

        # Store logs
        times.append(current_time)
        temps.append(measurement)
        outputs.append(heater_power)
        setpoints.append(setpoint)

    # Plot temperature response
    plt.figure(figsize=(10, 6))
    plt.plot(times, temps, label="Measured Temperature")
    plt.plot(times, setpoints, "--", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("PID Temperature Control Simulation")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    run_simulation()
