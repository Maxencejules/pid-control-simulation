import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from controller.pid import PID
from system.system_model import ThermalSystem


def main():
    dt = 0.1
    setpoint = 50.0

    system = ThermalSystem()
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)

    measurement = system.temperature
    heater_power = 0.0

    # Logging
    times = []
    temps = []
    powers = []

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.subplots_adjust(hspace=0.35)

    # Temperature graph
    temp_line, = ax1.plot([], [], label="Temperature")
    ax1.axhline(setpoint, linestyle="--", color="orange", label="Setpoint")
    ax1.set_xlim(0, 60)
    ax1.set_ylim(20, 60)
    ax1.set_title("Real-Time PID Temperature Control")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (°C)")
    ax1.legend()
    ax1.grid(True)

    # Power graph
    power_line, = ax2.plot([], [], label="Heater Power (0–1)")
    ax2.set_xlim(0, 60)
    ax2.set_ylim(0, 1.05)
    ax2.set_title("Heater Power Over Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Power")
    ax2.legend()
    ax2.grid(True)

    def update(frame):
        nonlocal measurement, heater_power

        t = frame * dt

        # PID control
        u = pid.compute(setpoint=setpoint, measurement=measurement)
        u = max(0.0, min(1.0, u))
        heater_power = u

        # Update plant
        measurement = system.update(heater_power, dt)

        # Log
        times.append(t)
        temps.append(measurement)
        powers.append(heater_power)

        # Update plot data
        temp_line.set_data(times, temps)
        power_line.set_data(times, powers)

        # Scroll window forward
        ax1.set_xlim(max(0, t - 60), t + 1)
        ax2.set_xlim(max(0, t - 60), t + 1)

        return temp_line, power_line

    anim = FuncAnimation(fig, update, frames=600, interval=100)
    plt.show()


if __name__ == "__main__":
    main()
