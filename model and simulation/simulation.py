import control as ct
import matplotlib.pyplot as plt
import numpy as np

# --- 1. System Parameters ---
M_stem = 0.006  # Stem mass [kg]
M_body = 0.048  # Body mass [kg]
M_motor = 0.112  # Motor mass [kg]
M_wheel = 0.109  # Wheel mass [kg]
H_stem = 0.044  # Stem height [m]
H_body = 0.095  # Body height [m]
D_motor = 0.042  # Motor diameter [m]
D_wheel_out = 0.126  # Outer wheel diameter [m]
D_wheel_in = 0.104  # Inner wheel diameter [m]
K_u = 1.0
g = 9.81

# --- 2. Derived Constants ---
M_total = M_stem + M_body + M_motor
R_out = D_wheel_out / 2
R_in = D_wheel_in / 2
L_com = (
    M_stem * (H_stem / 2)
    + M_body * (H_stem + H_body / 2)
    + M_motor * (H_stem + H_body + D_motor / 2)
) / M_total
I_wheel = 0.5 * M_wheel * (R_out**2 + R_in**2)
I_pendulum = (
    (1 / 3) * M_stem * H_stem**2
    + (1 / 3) * M_body * H_body**2
    + M_body * (H_stem + H_body / 2) ** 2
    + M_motor * (H_stem + H_body + D_motor / 2) ** 2
)

print("--- Calculated System Constants ---")
print(f"Pendulum Inertia (I_pendulum): {I_pendulum:.4f} kg*m^2")
print(f"Wheel Inertia (I_wheel): {I_wheel:.4f} kg*m^2")
print("-----------------------------------")

# --- 3. Digital LQR Controller Design (using a Reduced-Order Model) ---

# a. Define the full 4-state continuous-time model for simulation purposes
A_full = np.array(
    [
        [0, 1, 0, 0],
        [M_total * g * L_com / I_pendulum, 0, 0, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 0],
    ]
)
B_full = np.array([[0], [-K_u / I_pendulum], [0], [K_u / I_wheel]])

# b. Define the reduced 3-state model for controller design
# The state vector is x_reduced = [phi, phi_dot, psi_dot]
A_reduced = np.array([[0, 1, 0], [M_total * g * L_com / I_pendulum, 0, 0], [0, 0, 0]])
B_reduced = np.array([[0], [-K_u / I_pendulum], [K_u / I_wheel]])

# c. Discretize both systems
Ts = 0.01  # Sampling time
# Full system for simulation
sys_full_c = ct.ss(A_full, B_full, np.eye(4), 0)
sys_full_d = ct.c2d(sys_full_c, Ts, method="zoh")
Ad_full, Bd_full = sys_full_d.A, np.array(sys_full_d.B)

# Reduced system for controller design
sys_reduced_c = ct.ss(A_reduced, B_reduced, np.eye(3), 0)
sys_reduced_d = ct.c2d(sys_reduced_c, Ts, method="zoh")
Ad_reduced, Bd_reduced = sys_reduced_d.A, sys_reduced_d.B

# d. Define LQR cost matrices for the REDUCED system
Q_reduced = np.diag(
    [
        100.0,  # Cost for phi (tilt angle)
        1.0,  # Cost for phi_dot (tilt angular velocity)
        0.1,  # Cost for psi_dot (wheel velocity)
    ]
)
R = np.diag([0.01])  # Cost for control input u

# e. Calculate the optimal LQR gain K for the REDUCED system
# This call will now be numerically stable.
K, S, E = ct.dlqr(Ad_reduced, Bd_reduced, Q_reduced, R)

print(f"\n--- Digital LQR Controller (Reduced Order) ---")
print(f"Sampling Time (Ts): {Ts} s")
print(f"Optimal LQR Gain (K): {K}")  # This is now a 1x3 matrix
print("---------------------------------------------")


# --- 4. Discrete-Time Simulation (of the Full System) ---
sim_time = 1.0  # seconds
t_span = np.arange(0, sim_time, Ts)
num_steps = len(t_span)

# Initial conditions for the FULL 4-state system
initial_tilt_angle_deg = 5.0
x0 = np.array([np.deg2rad(initial_tilt_angle_deg), 0.0, 0.0, 0.0])

# Initialize arrays to store simulation history
x_history = np.zeros((num_steps, 4))
u_history = np.zeros(num_steps)
x_history[0, :] = x0
x_k = x0  # Current full state

# Run the simulation loop
for i in range(num_steps - 1):
    # Construct the reduced state vector from the full state
    x_k_reduced = np.array([x_k[0], x_k[1], x_k[3]])  # [phi, phi_dot, psi_dot]

    # Calculate control input using the reduced-order gain K
    u_k = -(K @ x_k_reduced)
    u_history[i] = u_k[0]

    # Update the FULL state using the full system dynamics
    x_k_plus_1 = Ad_full @ x_k + Bd_full.flatten() * u_k
    x_history[i + 1, :] = x_k_plus_1
    x_k = x_k_plus_1

# Store the final control input
x_k_reduced = np.array([x_k[0], x_k[1], x_k[3]])
u_history[-1] = -(K @ x_k_reduced)[0]

# --- 5. Visualization of Results ---
# (This section is identical to the previous script and will plot the 4 states)
# ... [Plotting code remains the same] ...
plt.style.use("seaborn-v0_8-whitegrid")
fig, axs = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
fig.suptitle("Digital LQR Control Simulation (Reduced-Order Design)", fontsize=16)

# Plot Pendulum Tilt Angle (phi)
axs[0].plot(t_span, np.rad2deg(x_history[:, 0]))
axs[0].set_ylabel("Tilt Angle ($\\phi$) [deg]")
axs[0].set_title("Pendulum Tilt Angle (Primary Objective)")
axs[0].grid(True)

# Plot Wheel Angular Velocity (psi_dot)
axs[1].plot(t_span, np.rad2deg(x_history[:, 3]))
axs[1].set_ylabel("Wheel Velocity ($\\dot{\\psi}$) [deg/s]")
axs[1].set_title("Reaction Wheel Angular Velocity (Secondary Objective)")
axs[1].grid(True)

# Plot Control Input (u)
axs[2].plot(t_span, u_history)
axs[2].set_ylabel("Control Input ($u$)")
axs[2].set_title("Controller Output Signal")
axs[2].grid(True)

# Plot Pendulum Angular Velocity (phi_dot)
axs[3].plot(t_span, np.rad2deg(x_history[:, 1]))
axs[3].set_ylabel("Angular Velocity ($\\dot{\\phi}$) [deg/s]")
axs[3].set_title("Pendulum Angular Velocity")
axs[3].grid(True)

# Plot Wheel Angle (psi)
axs[4].plot(t_span, np.rad2deg(x_history[:, 2]))
axs[4].set_ylabel("Wheel Angle ($\\psi$) [deg]")
axs[4].set_title("Reaction Wheel Angle (Uncontrolled)")
axs[4].grid(True)


axs[-1].set_xlabel("Time [s]")
plt.tight_layout(rect=(0, 0, 1, 0.96))
plt.show()
