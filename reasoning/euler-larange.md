Okay, let's derive the mathematical model for your self-balancing robot.

## 1. System Definition and Coordinates

* **$\theta$**: The angle of the robot's longitudinal axis with respect to the upward vertical axis. A positive $\theta$ could represent a tilt to the right (clockwise). This is our output.
* **$\phi$**: The angle of the reaction wheel relative to the robot's body.
* **$u = \ddot{\phi}$**: The angular acceleration of the reaction wheel relative to the motor shaft. This is our input.
* The absolute angular velocity of the wheel with respect to the inertial frame is $\dot{\alpha}_{wheel} = \dot{\theta} + \dot{\phi}$.
* The absolute angular acceleration of the wheel is $\ddot{\alpha}_{wheel} = \ddot{\theta} + \ddot{\phi} = \ddot{\theta} + u$.

We will use the Lagrangian mechanics approach. The Lagrangian $L$ is defined as $L = T - V$, where $T$ is the total kinetic energy and $V$ is the total potential energy of the system.

The equations of motion are given by Euler-Lagrange equations:
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = Q_i$$
where $q_i$ are the generalized coordinates ($\theta, \phi$) and $Q_i$ are the generalized non-conservative forces/torques.

## 2. Parameters and Constants

Let's define all the necessary parameters based on your description:

* **Body (Green Rectangle):**
    * $M_{body}$: Mass
    * $W_{body}$: Width
    * $H_{body}$: Height
* **Stem:**
    * $L_{stem}$: Length (massless)
* **Wheel (Blue Ring):**
    * $M_{wheel}$: Mass
    * $D_{wheel\_out}$: Outer diameter $\implies R_{wheel\_out} = D_{wheel\_out}/2$
    * $D_{wheel\_in}$: Internal diameter $\implies R_{wheel\_in} = D_{wheel\_in}/2$
    * Spokes are massless.
* **Motor (Red Circle):**
    * $M_{motor}$: Mass
    * $D_{motor}$: Diameter $\implies R_{motor} = D_{motor}/2$
* **Gravity:**
    * $g$: Acceleration due to gravity

## 3. Derived Geometric and Inertial Properties

* **Distances from Pivot (P):**
    * To CM of Body (green rectangle): $L_{CM\_body} = L_{stem} + H_{body}/2$
    * To CM of Motor (and axis of wheel): $L_{CM\_motor} = L_{stem} + H_{body}$
    * To CM of Wheel (same as motor): $L_{CM\_wheel} = L_{stem} + H_{body}$ (Let's call this $L_R$ for simplicity, $L_R = L_{stem} + H_{body}$)

* **Moments of Inertia (MoI):**
    * **Body (green rectangle) about its own CM** (axis $\perp$ to view):
        $I_{body,CM} = \frac{1}{12} M_{body} (H_{body}^2 + W_{body}^2)$
    * **Body (green rectangle) about the pivot P:** (using Parallel Axis Theorem)
        $I_{body,P} = I_{body,CM} + M_{body} L_{CM\_body}^2$
    * **Motor about its own CM** (axis $\perp$ to view, assuming it's a disk/cylinder):
        $I_{motor,CM} = \frac{1}{2} M_{motor} R_{motor}^2 = \frac{1}{2} M_{motor} (D_{motor}/2)^2$
    * **Motor about the pivot P:**
        $I_{motor,P} = I_{motor,CM} + M_{motor} L_{CM\_motor}^2$
    * **Total MoI of Robot Assembly (Body + Motor) about pivot P:**
        $I_{robot\_assembly} = I_{body,P} + I_{motor,P}$
        $I_{robot\_assembly} = \left( \frac{1}{12} M_{body} (H_{body}^2 + W_{body}^2) + M_{body} (L_{stem} + H_{body}/2)^2 \right) + \left( \frac{1}{2} M_{motor} (D_{motor}/2)^2 + M_{motor} (L_{stem} + H_{body})^2 \right)$
    * **Reaction Wheel about its own axis of rotation:** (as an annulus)
        $I_{wheel} = \frac{1}{2} M_{wheel} (R_{wheel\_out}^2 + R_{wheel\_in}^2)$

## 4. Kinetic Energy (T)

* **Kinetic Energy of Robot Assembly (Body + Motor) rotating about P:**
    $T_{robot\_assembly} = \frac{1}{2} I_{robot\_assembly} \dot{\theta}^2$
* **Kinetic Energy of Reaction Wheel:**
    * Translational (CM of wheel moving with the body): $v_{wheel,CM} = L_R \dot{\theta}$
        $T_{wheel,trans} = \frac{1}{2} M_{wheel} v_{wheel,CM}^2 = \frac{1}{2} M_{wheel} L_R^2 \dot{\theta}^2$
    * Rotational (about its own CM with absolute angular velocity $\dot{\alpha}_{wheel} = \dot{\theta} + \dot{\phi}$):
        $T_{wheel,rot} = \frac{1}{2} I_{wheel} (\dot{\theta} + \dot{\phi})^2 = \frac{1}{2} I_{wheel} (\dot{\theta}^2 + 2\dot{\theta}\dot{\phi} + \dot{\phi}^2)$
* **Total Kinetic Energy (T):**
    $T = T_{robot\_assembly} + T_{wheel,trans} + T_{wheel,rot}$
    $T = \frac{1}{2} I_{robot\_assembly} \dot{\theta}^2 + \frac{1}{2} M_{wheel} L_R^2 \dot{\theta}^2 + \frac{1}{2} I_{wheel} (\dot{\theta}^2 + 2\dot{\theta}\dot{\phi} + \dot{\phi}^2)$
    $T = \frac{1}{2} (I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \dot{\theta}^2 + I_{wheel} \dot{\theta}\dot{\phi} + \frac{1}{2} I_{wheel} \dot{\phi}^2$

## 5. Potential Energy (V)

Assuming $\theta=0$ is the upward vertical position (unstable equilibrium for an inverted pendulum). The height of a CM is $h = L_{CM} \cos\theta$.
* $V_{body} = M_{body} g L_{CM\_body} \cos\theta = M_{body} g (L_{stem} + H_{body}/2) \cos\theta$
* $V_{motor} = M_{motor} g L_{CM\_motor} \cos\theta = M_{motor} g (L_{stem} + H_{body}) \cos\theta$
* $V_{wheel} = M_{wheel} g L_{CM\_wheel} \cos\theta = M_{wheel} g (L_{stem} + H_{body}) \cos\theta$
* **Total Potential Energy (V):**
    $V = (M_{body}(L_{stem} + H_{body}/2) + (M_{motor} + M_{wheel})(L_{stem} + H_{body})) g \cos\theta$
    Let $M_{eff}L_{eff} = M_{body}(L_{stem} + H_{body}/2) + (M_{motor} + M_{wheel})(L_{stem} + H_{body})$.
    So, $V = M_{eff}L_{eff} g \cos\theta$.

## 6. Lagrangian (L) and Equations of Motion

$L = T - V$
$L = \frac{1}{2} (I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \dot{\theta}^2 + I_{wheel} \dot{\theta}\dot{\phi} + \frac{1}{2} I_{wheel} \dot{\phi}^2 - M_{eff}L_{eff} g \cos\theta$

Let $\tau_m$ be the torque applied by the motor onto the reaction wheel.
Then $Q_{\theta} = -\tau_m$ (reaction torque on the body) and $Q_{\phi} = \tau_m$.

**For coordinate $\theta$:**
$\frac{\partial L}{\partial \dot{\theta}} = (I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \dot{\theta} + I_{wheel} \dot{\phi}$
$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) = (I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \ddot{\theta} + I_{wheel} \ddot{\phi}$
$\frac{\partial L}{\partial \theta} = M_{eff}L_{eff} g \sin\theta$
So, $(I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \ddot{\theta} + I_{wheel} \ddot{\phi} - M_{eff}L_{eff} g \sin\theta = -\tau_m$  (Eq. 1)

**For coordinate $\phi$:**
$\frac{\partial L}{\partial \dot{\phi}} = I_{wheel} \dot{\theta} + I_{wheel} \dot{\phi}$
$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\phi}}\right) = I_{wheel} \ddot{\theta} + I_{wheel} \ddot{\phi}$
$\frac{\partial L}{\partial \phi} = 0$
So, $I_{wheel} \ddot{\theta} + I_{wheel} \ddot{\phi} = \tau_m$ (Eq. 2)

The input is $u = \ddot{\phi}$. Substitute $\ddot{\phi} = u$ into Eq. 2:
$\tau_m = I_{wheel} (\ddot{\theta} + u)$

Now substitute this $\tau_m$ and $\ddot{\phi} = u$ into Eq. 1:
$(I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \ddot{\theta} + I_{wheel} u - M_{eff}L_{eff} g \sin\theta = - (I_{wheel} (\ddot{\theta} + u))$
$(I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel}) \ddot{\theta} + I_{wheel} u - M_{eff}L_{eff} g \sin\theta = -I_{wheel}\ddot{\theta} - I_{wheel}u$

Rearranging to solve for $\ddot{\theta}$:
$(I_{robot\_assembly} + M_{wheel}L_R^2 + I_{wheel} + I_{wheel}) \ddot{\theta} + I_{wheel}u + I_{wheel}u - M_{eff}L_{eff} g \sin\theta = 0$
$(I_{robot\_assembly} + M_{wheel}L_R^2 + 2I_{wheel}) \ddot{\theta} + 2I_{wheel} u - M_{eff}L_{eff} g \sin\theta = 0$

## 7. Final Mathematical Model

The equation of motion relating the input $u = \ddot{\phi}$ (wheel's angular acceleration relative to the motor shaft) to the output $\theta$ (robot's tilt angle) is:

$$(I_{robot\_assembly} + M_{wheel}L_R^2 + 2I_{wheel}) \ddot{\theta} = M_{eff}L_{eff} g \sin\theta - 2I_{wheel} u$$

So, the angular acceleration of the robot body is:

$$\ddot{\theta} = \frac{M_{eff}L_{eff} g \sin\theta - 2I_{wheel} u}{I_{robot\_assembly} + M_{wheel}L_R^2 + 2I_{wheel}}$$

Where:
* $\ddot{\theta}$ is the angular acceleration of the robot's body.
* $\theta$ is the angle of the robot's body from the vertical.
* $u = \ddot{\phi}$ is the angular acceleration of the reaction wheel relative to the robot's body (the input).
* $g$ is the acceleration due to gravity.

And the combined terms are:
* **Denominator (Total effective inertia for $\theta$ motion):**
    $I_{total\_eff} = I_{robot\_assembly} + M_{wheel}L_R^2 + 2I_{wheel}$
    $I_{robot\_assembly} = \left( \frac{1}{12} M_{body} (H_{body}^2 + W_{body}^2) + M_{body} (L_{stem} + H_{body}/2)^2 \right) + \left( \frac{1}{2} M_{motor} (D_{motor}/2)^2 + M_{motor} (L_{stem} + H_{body})^2 \right)$
    $L_R = L_{stem} + H_{body}$
    $I_{wheel} = \frac{1}{2} M_{wheel} ((D_{wheel\_out}/2)^2 + (D_{wheel\_in}/2)^2)$

* **Numerator (Gravitational torque term):**
    $M_{eff}L_{eff} = M_{body}(L_{stem} + H_{body}/2) + (M_{motor} + M_{wheel})(L_{stem} + H_{body})$

This second-order non-linear ordinary differential equation represents the mathematical model of your self-balancing robot. For small angles $\theta$, $\sin\theta \approx \theta$, which can be used for linearization if needed for controller design.