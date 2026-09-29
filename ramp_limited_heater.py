"""Display the 0.1 power units/second actuator slew experiment."""
from demo import show_experiment


def run_ramp_limited_control():
    show_experiment("ramp_limited_heater")


if __name__ == "__main__":
    run_ramp_limited_control()
