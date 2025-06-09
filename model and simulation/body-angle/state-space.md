From the model,

$\ddot{\theta}(t) = \frac{-3.900 \sin(\theta(t)) - 0.0364 \ddot{\phi}(t)}{0.6496}$

This is a non-linear equation due to the $\sin(\theta)$ term. To linearize it, we can assume that the robot will be operating near the upright position, so $\theta$ will be small. For small angles, we can use the approximation $\sin(\theta) \approx \theta$.

The linearized equation becomes:

$\ddot{\theta}(t) \approx \frac{-3.900 \theta(t) - 0.0364 \ddot{\phi}(t)}{0.6496}$
$\ddot{\theta}(t) \approx -6.00 \theta(t) - 0.056 \ddot{\phi}(t)$

Now, we can create the state-space representation.

* **States**: We need two states for this second-order system. Let's choose the body angle and body angular velocity: $x_b = \begin{bmatrix} \theta \\ \dot{\theta} \end{bmatrix}$.
* **Input**: The input is the wheel's angular acceleration, $u_b = \ddot{\phi}(t)$.
* **Output**: Let's choose the body angle as the output, $y_b = \theta$.

The state-space representation is:

$\dot{x}_b = A_b x_b + B_b u_b$
$y_b = C_b x_b + D_b u_b$

Where:
$A_b = \begin{bmatrix} 0 & 1 \\ -6.00 & 0 \end{bmatrix}$
$B_b = \begin{bmatrix} 0 \\ -0.056 \end{bmatrix}$
$C_b = [1 \quad 0]$
$D_b = [0]$

This gives us the state-space model for the body angle:

$$
\begin{gather*}
\dot{x}_b = \begin{bmatrix} 0 & 1 \\ -6.00 & 0 \end{bmatrix} x_b + \begin{bmatrix} 0 \\ -0.056 \end{bmatrix} u_b \\
y_b = [1 \quad 0] x_b + [0] u_b
\end{gather*}
$$
