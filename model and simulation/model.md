# System geometry and variables

Consider the pendulum link (stem + body + motor) of total mass $M_{\rm total}=M_{\rm stem}+M_{\rm body}+M_{\rm motor}$, pivoted at the stem’s base. Let $\phi$ be the tilt angle of the pendulum from vertical (positive CCW about the horizontal tilt axis). Let $\psi$ be the rotation angle of the inertia wheel (driven by the motor) relative to the pendulum. The wheel has mass $M_{\rm wheel}$ and (outer/inner) radii $R_{\rm out}=D_{\rm wheel_{\rm out}}/2$, $R_{\rm in}=D_{\rm wheel_{\rm in}}/2$. Its moment of inertia about its horizontal axis is that of a thin ring: $I_{\rm wheel}=\frac{1}{4}M_{\rm wheel}(R_{\rm out}^2 + R_{\rm in}^2)$.

Assuming no slipping and no front-back motion, the only restoring torque is from gravity on the pendulum’s center of mass (COM). The COM height above the pivot is $L_{\rm com}=\frac{M_{\rm stem}\frac{H_{\rm stem}}{2}+M_{\rm body}\bigl(H_{\rm stem}+\tfrac{H_{\rm body}}{2}\bigr)+M_{\rm motor}\bigl(H_{\rm stem}+H_{\rm body}+\tfrac{D_{\rm motor}}{2}\bigr)}{M_{\rm total}}.$

The total inertia of the pendulum about the tilt axis is $I_{\rm pendulum} = \frac{1}{3}M_{\rm stem}H_{\rm stem}^2 + \Bigl(\frac{1}{3}M_{\rm body}H_{\rm body}^2 + M_{\rm body}H_{\rm stem}^2\Bigr) + M_{\rm motor}(H_{\rm stem}+H_{\rm body}+\tfrac{D_{\rm motor}}{2})^2, $ where the first term is a slender-rod formula for the stem, the second combines the body’s inertia about its base and its offset, and the third treats the motor mass as a point at its center. (Any small rotor inertia of the motor can be lumped into the reaction wheel inertia if needed.)

# Nonlinear equations of motion

Using Lagrange’s equations (or Newton’s laws) yields coupled torques on the pendulum and wheel. Let $\tau$ be the torque exerted by the motor on the wheel (positive when the wheel spins CCW). By action–reaction, the wheel exerts $-\tau$ on the pendulum. Including gravity, the nonlinear equations are:

* **Pendulum:** $I_{\rm pendulum}\ddot\phi = M_{\rm total}gL_{\rm com}\sin\phi -\tau.$
* **Wheel:**   $I_{\rm wheel}\ddot\psi = \tau.$

Here $I_{\rm pendulum}$ and $I_{\rm wheel}$ are as defined above, and $M_{\rm total}gL_{\rm com}\sin\phi$ is the gravitational torque (positive for $\phi>0$ as the pendulum tips further). In words, the wheel’s motor torque $\tau$ accelerates the wheel and produces an equal-and-opposite torque on the pendulum.

# Linearization about upright

For control design we linearize around the upright equilibrium $\phi=0$. For small $\phi$, $\sin\phi\approx\phi$. Neglecting damping and friction, the linearized equations are: $I_{\rm pendulum}\ddot\phi = M_{\rm total}gL_{\rm com}\phi -\tau, I_{\rm wheel}\ddot\psi = \tau.$ Equivalently, one can write $I_{\rm pendulum}\ddot\phi - M_{\rm total}gL_{\rm com}\phi + \tau = 0, I_{\rm wheel}\ddot\psi - \tau = 0.$ In this form the gravitational term $M_{\rm total}gL_{\rm com},\phi$ acts as a linear “negative stiffness” (destabilizing torque) about the upright position. These coupled linear equations match the standard reaction-wheel inverted-pendulum model.

# Including motor dynamics

With supply voltage $V$ and armature resistance $R$, the motor torque is $\tau = K_ti$ and the back-EMF is $K_e\dot\psi$, yielding (neglecting inductance) $i = \frac{V - K_e\dot\psi}{R}, \tau = K_ti \approx \frac{K_t}{R}V - \frac{K_tK_e}{R}\dot\psi.$ Using control input $u\in[-1,1]$ to represent the normalized voltage (so $V=V_{\max}u$), we may simply write $\tau = K_uu$ for some constant $K_u$ (and possibly include a small feedback term in $\dot\psi$ if needed). The essential model remains $I_{\rm pendulum}\ddot\phi = M_{\rm total}gL_{\rm com}\phi - K_u u, I_{\rm wheel}\ddot\psi = K_u u.$ Thus, in linear state-space form one can take states $x=[\phi,\dot\phi,\psi,\dot\psi]^T$ and write:

$$
\begin{pmatrix}\dot\phi\\ \ddot\phi\\ \dot\psi\\ \ddot\psi\end{pmatrix} =
\begin{pmatrix}
0 & 1 & 0 & 0\\
\frac{M_{\rm total}gL_{\rm com}}{I_{\rm pendulum}} & 0 & 0 & -\frac{1}{I_{\rm pendulum}}\\
0 & 0 & 0 & 1\\
0 & 0 & 0 & 0
\end{pmatrix}
\begin{pmatrix}\phi\\ \dot\phi\\ \psi\\ \dot\psi\end{pmatrix} +
\begin{pmatrix}0\\ \frac{K_u}{I_{\rm pendulum}}\\ 0\\ \frac{K_u}{I_{\rm wheel}}\end{pmatrix}u.
$$

Any equivalent linear form is acceptable. In summary, the **symbolic linear model** is:

* $I_{\rm pendulum}\ddot\phi = M_{\rm total}gL_{\rm com},\phi - \tau$
* $I_{\rm wheel}\ddot\psi = \tau$

with $\tau=K_u u$ as the control torque.
