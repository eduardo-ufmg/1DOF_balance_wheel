import json
import os

import control
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Digital controller design for RWiP: stabilize body angle phi to zero
# Sampling and control period
Ts = 0.01  # seconds


def parse_value(val):
    return float(val.split()[0])


def load_parameters(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    M_wheel = parse_value(data["wheel_mass"])
    D_out = parse_value(data["wheel_diameter_out"])
    D_in = parse_value(data["wheel_diameter_in"])
    M_motor = parse_value(data["motor_mass"])
    D_motor = parse_value(data["motor_diameter"])
    M_body = parse_value(data["body_mass"])
    H_body = parse_value(data["body_height"])
    M_stem = parse_value(data["stem_mass"])
    H_stem = parse_value(data["stem_height"])
    K_u = parse_value(data["ku"])

    # geometry and inertias
    R_out = D_out / 2
    R_in = D_in / 2
    I_wheel = 0.25 * M_wheel * (R_out**2 + R_in**2)
    M_total = M_stem + M_body + M_motor
    L_com = (
        M_stem * (H_stem / 2)
        + M_body * (H_stem + H_body / 2)
        + M_motor * (H_stem + H_body + D_motor / 2)
    ) / M_total
    # pendulum inertia
    I_p = (
        (1 / 3) * M_stem * H_stem**2
        + (1 / 3) * M_body * H_body**2
        + M_body * H_stem**2
        + M_motor * (H_stem + H_body + D_motor / 2) ** 2
    )
    g = 9.81
    return M_total, L_com, I_p, I_wheel, K_u, g


def build_state_space(params):
    M_total, L_com, I_p, I_wheel, K_u, g = params
    # state x = [phi, phi_dot, psi, psi_dot]
    A = np.zeros((4, 4))
    A[0, 1] = 1
    A[1, 0] = M_total * g * L_com / I_p
    A[2, 3] = 1
    # no natural coupling to psi
    B = np.zeros((4, 1))
    B[1, 0] = -K_u / I_p
    B[3, 0] = K_u / I_wheel
    return A, B


def design_digital_lqr(A, B, Ts):
    # continuous to discrete
    sysc = control.ss(A, B, np.eye(4), np.zeros((4, 1)))
    sysd = control.c2d(sysc, Ts)
    Ad, Bd = np.array(sysd.A), np.array(sysd.B)

    # cost: penalize phi heavily
    Q = np.diag([1000, 1, 1, 1])
    R = np.array([[1]])
    # discrete LQR
    K, S, E = control.dlqr(Ad, Bd, Q, R)
    return Ad, Bd, K


def simulate_closed_loop(Ad, Bd, K, x0, steps):
    x = np.array(x0).reshape(-1, 1)
    X = [x.flatten()]
    for _ in range(steps):
        u = -K.dot(x)
        x = Ad.dot(x) + Bd.dot(u)
        X.append(x.flatten())
    return np.array(X)


def main():
    # load params and build model
    script_dir = os.path.dirname(__file__)
    params = load_parameters(
        os.path.join(script_dir, "../identification/measurements.json")
    )
    A, B = build_state_space(params)

    # design digital LQR controller
    Ad, Bd, K = design_digital_lqr(A, B, Ts)
    print("State-feedback gain K:", K)

    # simulate for 5 seconds
    total_time = 5.0
    steps = int(total_time / Ts)
    x0 = [0.1, 0.0, 0.0, 0.0]  # initial tilt

    X = simulate_closed_loop(Ad, Bd, K, x0, steps)
    t = np.arange(0, total_time + Ts, Ts)

    phi = X[:, 0]
    psi = X[:, 2]

    plt.figure()
    plt.plot(t, phi)
    plt.title("Pendulum Angle (phi) with LQR Controller")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")

    plt.figure()
    plt.plot(t, psi)
    plt.title("Wheel Angle (psi) with LQR Controller")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")

    plt.show()


if __name__ == "__main__":
    main()
