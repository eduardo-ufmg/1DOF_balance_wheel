From the model,

$\ddot{\phi}(t) \approx 42.0 D(t) - 31.3 \dot{\phi}(t) - \ddot{\theta}(t)$

To put this equation in a State Space format, we can define the state, input, and output.

* **State**: Let the state be the wheel's angular velocity relative to the motor shaft, $x_m = \dot{\phi}$.
* **Inputs**: The inputs are the duty cycle $D(t)$ and the body's angular acceleration $\ddot{\theta}(t)$. We can represent this as a vector $u_m = \begin{bmatrix} D(t) \\ \ddot{\theta}(t) \end{bmatrix}$.
* **Output**: The output is the wheel's angular velocity, $y_m = \dot{\phi}$.

The state-space representation is:

$\dot{x}_m = A_m x_m + B_m u_m$
$y_m = C_m x_m + D_m u_m$

Where:
$A_m = [-31.3]$
$B_m = [42.0 \quad -1]$
$C_m = [1]$
$D_m = [0 \quad 0]$

This gives us the state-space model for the motor acceleration:

$$
\begin{gather*}
\dot{x}_m = [-31.3] x_m + [42.0 \quad -1] u_m \\
y_m = [1] x_m + [0 \quad 0] u_m
\end{gather*}
$$
