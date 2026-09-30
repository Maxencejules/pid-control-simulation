"""Display the 30 -> 50 -> 40 Celsius setpoint experiment."""
from demo import show_experiment
from experiments import step_setpoint


def run_step_response():
    show_experiment("step_response")


if __name__ == "__main__":
    run_step_response()
