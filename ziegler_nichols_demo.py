import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def simulate_p_controller(kp: float, dt: float, sim_time: float):
    steps = int(sim_time / dt)
    setpoint = 50.0
    system = ThermalSystem()
    pid = PID(kp=kp, ki=0.0, kd=0.0, dt=dt)

    times, temps = [], []
    measurement = system.temperature

    for i in range(steps):
        t = i * dt
        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)
        heater_power = max(0.0, min(1.0, heater_power))
        measurement = system.update(heater_power, dt)

        times.append(t)
        temps.append(measurement)

    return times, temps


def simulate_pid(kp: float, ki: float, kd: float, dt: float, sim_time: float):
    steps = int(sim_time / dt)
    setpoint = 50.0
    system = ThermalSystem()
    pid = PID(kp=kp, ki=ki, kd=kd, dt=dt)

    times, temps = [], []
    measurement = system.temperature

    for i in range(steps):
        t = i * dt
        heater_power = pid.compute(setpoint=setpoint, measurement=measurement)
        heater_power = max(0.0, min(1.0, heater_power))
        measurement = system.update(heater_power, dt)

        times.append(t)
        temps.append(measurement)

    return times, temps


def run_ziegler_nichols_demo():
    dt = 0.1
    sim_time = 120.0

    # Example "ultimate gain" and period (found experimentally)
    Ku = 5.0   # ultimate gain (causes sustained oscillations)
    Tu = 20.0  # ultimate period (approx seconds)

    # Ziegler–Nichols classic PID tuning
    Kp = 0.6 * Ku
    Ki = 2 * Kp / Tu
    Kd = Kp * Tu / 8

    t_p, temp_p = simulate_p_controller(Ku, dt, sim_time)
    t_pid, temp_pid = simulate_pid(Kp, Ki, Kd, dt, sim_time)

    plt.figure(figsize=(12, 6))
    plt.plot(t_p, temp_p, label=f"P-only at Ku={Ku} (sustained oscillations)")
    plt.plot(t_pid, temp_pid, label="PID tuned via Ziegler–Nichols")
    plt.axhline(50.0, linestyle="--", color="black", label="Setpoint")

    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("Ziegler–Nichols Tuning Demonstration")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_ziegler_nichols_demo()
