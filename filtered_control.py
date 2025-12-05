import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem
from utils.filtering import MovingAverageFilter


def run_filtered_control():
    dt = 0.1
    sim_time = 180.0
    steps = int(sim_time / dt)

    system = ThermalSystem(noise_std=0.5)  # noisier to show benefit
    pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)
    filt = MovingAverageFilter(window_size=10)

    setpoint = 50.0

    times, raw_temps, filt_temps, outputs = [], [], [], []
    measurement = system.temperature

    for i in range(steps):
        t = i * dt

        # Raw noisy measurement
        raw = system.update(heater_power=0.0, dt=dt)

        # Filtered measurement
        filtered = filt.filter(raw)

        heater_power = pid.compute(setpoint=setpoint, measurement=filtered)
        heater_power = max(0.0, min(1.0, heater_power))

        # Apply heater for next state
        measurement = system.update(heater_power, dt)

        times.append(t)
        raw_temps.append(raw)
        filt_temps.append(filtered)
        outputs.append(heater_power)

    plt.figure(figsize=(12, 10))

    plt.subplot(2, 1, 1)
    plt.plot(times, raw_temps, alpha=0.4, label="Raw Measurement")
    plt.plot(times, filt_temps, label="Filtered Measurement (MA)")
    plt.axhline(50.0, linestyle="--", color="orange", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("Effect of Moving Average Filtering on Noisy Sensor")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(times, outputs, label="Heater Power (0–1)")
    plt.xlabel("Time (s)")
    plt.ylabel("Power")
    plt.title("Heater Output With Filtered Feedback")
    plt.ylim(0, 1.05)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_filtered_control()
