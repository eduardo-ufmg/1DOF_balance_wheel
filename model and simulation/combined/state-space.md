For controller design, it's most effective to have a single state-space model that represents the entire system. We can achieve this by combining the two models.

Let's define the state vector for the combined system as:

$x = \begin{bmatrix} \theta \\ \dot{\theta} \\ \dot{\phi} \end{bmatrix}$

The input to the combined system is the duty cycle, $u = D(t)$. The outputs can be the body angle and the wheel's relative angular velocity, $y = \begin{bmatrix} \theta \\ \dot{\phi} \end{bmatrix}$.

After substituting the expression for $\ddot{\theta}$ from the body-angle model into the motor-acceleration model and rearranging, we get the following state-space representation:

$\dot{x} = Ax + Bu$
$y = Cx + Du$

Where:
$A = \begin{bmatrix} 0 & 1 & 0 \\ -6.356 & 0 & 1.86 \\ 6.35 & 0 & -33.2 \end{bmatrix}$

$B = \begin{bmatrix} 0 \\ -2.49 \\ 44.5 \end{bmatrix}$

$C = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$

$D = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$

This gives us the combined state-space model for the system:

$$
\begin{gather*}
\dot{x} = \begin{bmatrix} 0 & 1 & 0 \\ -6.356 & 0 & 1.86 \\ 6.35 & 0 & -33.2 \end{bmatrix} x + \begin{bmatrix} 0 \\ -2.49 \\ 44.5 \end{bmatrix} u \\
y = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} x + \begin{bmatrix} 0 \\ 0 \end{bmatrix} u
\end{gather*}
$$
