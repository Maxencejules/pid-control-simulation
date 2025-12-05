import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem
from utils.filtering import MovingAverageFilter


def run_filtered_control():
    dt = 0.1
    sim_time = 180.0
    steps = int(sim_time / dt)

    system = ThermalSystem(noise_std=0.5)  # noisier sensor to show filtering benefit
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)
    filt = MovingAverageFilter(window_size=10)

    setpoint = 50.0

    times, raw_temps, filt_temps, outputs = [], [], [], []

    measurement = system.temperature

    for i in range(steps):
        t = i * dt

        # Raw noisy measurement (sensor reading)
        raw = system.update(heater_power=0.0, dt=dt)

        # Filtered measurement using moving average
        filtered = filt.filter(raw)

        # PID uses the filtered signal
        heater_power = pid.compute(setpoint=setpoint, measurement=filtered)
        heater_power = max(0.0, min(1.0, heater_power))

        # Update plant with heater power
        measurement = system.update(heater_power, dt)

        # Log data
        times.append(t)
        raw_temps.append(raw)
        filt_temps.append(filtered)
        outputs.append(heater_power)

    # --- CLEAN subplot layout (2 rows) ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # --- Temperature subplot ---
    ax1 = axes[0]
    ax1.plot(times, raw_temps, alpha=0.4, label="Raw Measurement")
    ax1.plot(times, filt_temps, label="Filtered Measurement (MA)")
    ax1.axhline(50.0, linestyle="--", color="orange", label="Setpoint")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("Effect of Moving Average Filtering on Noisy Sensor")
    ax1.legend()
    ax1.grid(True)

    # --- Heater power subplot ---
    ax2 = axes[1]
    ax2.plot(times, outputs, label="Heater Power (0–1)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Power")
    ax2.set_title("Heater Output With Filtered Feedback")
    ax2.set_ylim(0, 1.05)
    ax2.legend()
    ax2.grid(True)

    # Layout fixes so titles do not overlap
    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.35)

    plt.show()


if __name__ == "__main__":
    run_filtered_control()
