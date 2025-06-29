# %% [markdown]
# # Inverted Pendulum with Reaction Wheel Simulation and Digital Controller
# This notebook simulates the linearized dynamics of an inverted pendulum balanced by a reaction inertia wheel,
# and designs a discrete-time state-feedback controller.

# %% [markdown]
# ## 1. Import Libraries

import matplotlib.pyplot as plt

# %%
import numpy as np
from scipy.integrate import odeint
from scipy.signal import cont2discrete, dlsim, dlti, place_poles

# %% [markdown]
# ## 2. Define System Parameters

# %%
# Physical parameters
g = 9.81  # m/s^2
M_stem, H_stem = 0.006, 0.044  # kg, m
M_body, H_body = 0.048, 0.095  # kg, m
M_motor, D_motor = 0.112, 0.042  # kg, m
M_wheel, R_out, R_in = 0.109, 0.63, 0.52  # kg, m

# Composite parameters
M_total = M_stem + M_body + M_motor
L_com = (
    M_stem * (H_stem / 2)
    + M_body * (H_stem + H_body / 2)
    + M_motor * (H_stem + H_body + D_motor / 2)
) / M_total
I_stem = (1 / 3) * M_stem * H_stem**2
I_body = (1 / 3) * M_body * H_body**2 + M_body * H_stem**2
I_motor = M_motor * (H_stem + H_body + D_motor / 2) ** 2
I_p = I_stem + I_body + I_motor
I_w = 0.25 * M_wheel * (R_out**2 + R_in**2)
K_u = 1  # control torque gain

# %% [markdown]
# ## 3. Continuous State-Space Model
# States: x = [phi, phi_dot, psi, psi_dot]

# %%
A_c = np.array(
    [[0, 1, 0, 0], [M_total * g * L_com / I_p, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]]
)
B_c = np.array([[0], [-K_u / I_p], [0], [K_u / I_w]])

# %% [markdown]
# ## 4. Discretization
# Sampling period Ts = 0.01 s

# %%
Ts = 0.01  # s
sysd = cont2discrete((A_c, B_c, np.eye(4), np.zeros((4, 1))), Ts, method="zoh")
A_d, B_d, _, _, _ = sysd

# %% [markdown]
# ## 5. State-Feedback Controller Design
# Place discrete poles for stability

# %%
# Desired closed-loop poles inside unit circle
desired_poles = np.array([0.9, 0.85, 0.8, 0.75])
place_obj = place_poles(A_d, B_d, desired_poles)
K_d = place_obj.gain_matrix  # state-feedback gain

# %% [markdown]
# ## 6. Simulation: Closed-Loop Response

# %%
T_final = 5.0
N = int(T_final / Ts)
# Preallocate
x = np.zeros((4, N))
t = np.linspace(0, T_final, N)
# Initial condition: small tilt
x[:, 0] = [0.1, 0, 0, 0]

# Simulation loop
for k in range(N - 1):
    # control law u = -K_d x
    u = -K_d.dot(x[:, k])
    # discrete update
    x[:, k + 1] = A_d.dot(x[:, k]) + B_d.flatten() * u

phi = x[0, :]
psi = x[2, :]

# %% [markdown]
# ## 7. Plot Closed-Loop Results

# %%
plt.figure()
plt.plot(t, phi, label=r"Pendulum Angle $\phi$ (rad)")
plt.plot(t, psi, label=r"Wheel Angle $\psi$ (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Closed-Loop Response with Digital State-Feedback")
plt.legend()
plt.grid(True)
plt.show()

# %% [markdown]
# ## 8. Remarks
# - We designed a discrete-time state-feedback controller via pole placement.
# - All states are assumed measured at Ts = 0.01 s and the control is updated at Tc = 0.01 s.
