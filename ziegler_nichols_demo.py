"""Explain why the original claimed ultimate-gain experiment was invalid.

This delay-free continuous first-order plant has no finite ultimate gain under
P control. The retained filename now displays the Euler sampling artifact
instead of inventing Ku and Tu or claiming a valid Ziegler-Nichols tuning.
"""
from demo import show_experiment


def run_ziegler_nichols_demo():
    print("No finite continuous Ku: pole = -(1/tau + gain*Kp) < 0 for Kp >= 0.")
    print("At dt=0.1s, Euler P-control stability requires Kp < 4.9791667.")
    show_experiment("sampling_limit")


if __name__ == "__main__":
    run_ziegler_nichols_demo()
