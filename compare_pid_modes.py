import matplotlib.pyplot as plt
from controller.pid import PID
from system.system_model import ThermalSystem


def simulate_controller(controller_type: str, dt: float, sim_time: float):
    steps = int(sim_time / dt)
    setpoint = 50.0

    # Independent plant for each controller
    system = ThermalSystem()

    # Controller selection
    if controller_type == "P":
        pid = PID(kp=2.0, ki=0.0, kd=0.0, dt=dt)
    elif controller_type == "PI":
        pid = PID(kp=2.0, ki=0.5, kd=0.0, dt=dt)
    elif controller_type == "PID":
        pid = PID(kp=3.0, ki=1.0, kd=0.2, dt=dt)
    else:
        raise ValueError("Unknown controller type")

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


def run_comparison():
    dt = 0.1
    sim_time = 120.0

    # Run three independent controllers
    times_p, temps_p = simulate_controller("P", dt, sim_time)
    times_pi, temps_pi = simulate_controller("PI", dt, sim_time)
    times_pid, temps_pid = simulate_controller("PID", dt, sim_time)

    # Clean figure layout
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(times_p, temps_p, label="P only")
    ax.plot(times_pi, temps_pi, label="PI")
    ax.plot(times_pid, temps_pid, label="PID (with anti-windup)")
    ax.axhline(50.0, linestyle="--", color="black", label="Setpoint")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("Comparison of P, PI, and PID Control")
    ax.legend()
    ax.grid(True)

    # Layout fixes
    fig.tight_layout()
    fig.subplots_adjust(top=0.9)

    plt.show()


if __name__ == "__main__":
    run_comparison()
