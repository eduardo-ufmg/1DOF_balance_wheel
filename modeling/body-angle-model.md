We will use the Lagrangian method or Newton-Euler method. Let's opt for a Newton-Euler approach for clarity in force and torque interactions, then verify with angular momentum principles. The angle $\theta$ is defined as the angle between the robot's longitudinal axis and the vertical axis. We'll assume $\theta$ is positive for a counter-clockwise tilt from the upward vertical. The input is $\ddot{\phi}$, the angular acceleration of the wheel relative to the motor shaft (body).

## 1. Definitions and Parameters

**Given Masses and Dimensions:**
* Body: $M_{body}$, $W_{body}$, $H_{body}$, $L_{stem}$
* Wheel: $M_{wheel}$, $D_{wheel\_out}$, $D_{wheel\_in}$
* Motor: $M_{motor}$, $D_{motor}$
* Acceleration due to gravity: $g$

**Derived Geometric Properties:**
* Radius of motor: $R_{motor} = D_{motor}/2$
* Outer radius of wheel: $R_{wheel\_out} = D_{wheel\_out}/2$
* Inner radius of wheel: $R_{wheel\_in} = D_{wheel\_in}/2$
* Distance from pivot to CoM of body:
    $L_b = L_{stem} + H_{body}/2$
* Distance from pivot to CoM of motor and CoM of wheel (axle location):
    $L_m = L_{stem} + H_{body} + R_{motor}$ (since motor center is $D_{motor}/2$ above the top of the body)

**Moments of Inertia (about respective CoMs, axis perpendicular to the plane of motion):**
* Body (rectangle): $I_{b,cm} = \frac{1}{12} M_{body} (H_{body}^2 + W_{body}^2)$
* Motor (disk): $I_{m,cm} = \frac{1}{2} M_{motor} R_{motor}^2 = \frac{1}{8} M_{motor} D_{motor}^2$
* Wheel (hollow cylinder/ring): $I_{w,cm} = \frac{1}{2} M_{wheel} (R_{wheel\_out}^2 + R_{wheel\_in}^2) = \frac{1}{8} M_{wheel} (D_{wheel\_out}^2 + D_{wheel\_in}^2)$

## 2. Equations of Motion

Let $\theta$ be the angle of the robot body from the vertical (positive counter-clockwise).
Let $\phi$ be the angle of the wheel relative to the robot body (positive counter-clockwise).
The absolute angular position of the wheel is $\theta_w = \theta + \phi$.
The absolute angular velocity of the wheel is $\dot{\theta}_w = \dot{\theta} + \dot{\phi}$.
The absolute angular acceleration of the wheel is $\ddot{\theta}_w = \ddot{\theta} + \ddot{\phi}$.

Let $\tau_m$ be the torque applied by the motor *to the wheel*. By Newton's third law, a torque of $-\tau_m$ is applied by the motor *to the robot body*.

**Equation for the Robot Body assembly (Body + Motor + Wheel structure):**
The "robot body assembly" consists of the physical body, the motor casing, and the mass of the wheel considered as if it were rigidly attached for calculating its contribution to the assembly's moment of inertia due to its CoM's motion.
The total moment of inertia of this assembly about the pivot point O ($I_{assembly, O}$) is:
$I_{assembly,O} = (I_{b,cm} + M_{body}L_b^2) + (I_{m,cm} + M_{motor}L_m^2) + M_{wheel}L_m^2$
The sum of torques about the pivot point O acting on the robot body assembly is:
$\sum \tau_{O, assembly} = I_{assembly,O} \ddot{\theta}$
The torques are:
1.  Gravitational torque: The CoM of the combined (body + motor + wheel) system needs to be found. Alternatively, sum individual gravitational torques.
    Let $L_{CoM,body} = M_{body}gL_b\sin\theta$ (clockwise if $\theta>0$, so negative)
    Let $L_{CoM,motor} = M_{motor}gL_m\sin\theta$ (clockwise if $\theta>0$, so negative)
    Let $L_{CoM,wheel} = M_{wheel}gL_m\sin\theta$ (clockwise if $\theta>0$, so negative)
    Total gravitational torque $\tau_g = -(M_{body}L_b + (M_{motor}+M_{wheel})L_m)g\sin\theta$.
    Let $M_{eff}L_{eff} = M_{body}L_b + (M_{motor}+M_{wheel})L_m$.
    So, $\tau_g = -M_{eff}L_{eff}g\sin\theta$.
2.  Reaction torque from motor: $-\tau_m$.

So, the equation for the robot body assembly's rotation is:
$I_{assembly,O} \ddot{\theta} = -M_{eff}L_{eff}g\sin\theta - \tau_m$  (Equation 1)

**Equation for the Wheel:**
The rotation of the wheel about its own CoM (which is the motor shaft) is governed by:
$\sum \tau_{wheel, cm} = I_{w,cm} \ddot{\theta}_w$
The only torque acting on the wheel to change its angular momentum about its own axis is $\tau_m$.
So, $\tau_m = I_{w,cm} \ddot{\theta}_w = I_{w,cm}(\ddot{\theta} + \ddot{\phi})$  (Equation 2)

**Combining the Equations:**
The input to the system is $u = \ddot{\phi}$ (the wheel's angular acceleration relative to the body).
Substitute $\ddot{\phi} = u$ into Equation 2:
$\tau_m = I_{w,cm}(\ddot{\theta} + u)$ (Equation 3)

Now substitute this expression for $\tau_m$ into Equation 1:
$I_{assembly,O} \ddot{\theta} = -M_{eff}L_{eff}g\sin\theta - I_{w,cm}(\ddot{\theta} + u)$
$I_{assembly,O} \ddot{\theta} + I_{w,cm}\ddot{\theta} = -M_{eff}L_{eff}g\sin\theta - I_{w,cm}u$
$(I_{assembly,O} + I_{w,cm}) \ddot{\theta} = -M_{eff}L_{eff}g\sin\theta - I_{w,cm}u$

This equation relates the robot's body tilt acceleration $\ddot{\theta}$ (derivative of the output) to the body's tilt angle $\theta$ and the input wheel acceleration $u = \ddot{\phi}$.

## 3. Mathematical Model

The mathematical model is a second-order nonlinear ordinary differential equation:

$$(I_{assembly,O} + I_{w,cm}) \ddot{\theta}(t) = -M_{eff}L_{eff}g\sin\theta(t) - I_{w,cm}\ddot{\phi}(t)$$

Where:
* $\theta(t)$ is the angle of the robot's longitudinal axis with the plane vertical axis (output).
* $\ddot{\phi}(t)$ is the wheel's angular acceleration relative to the motor shaft (input, denoted as $u(t)$).
* $g$ is the acceleration due to gravity.

The composite inertial terms are:
* $L_b = L_{stem} + H_{body}/2$
* $L_m = L_{stem} + H_{body} + D_{motor}/2$
* $I_{b,cm} = \frac{1}{12} M_{body} (H_{body}^2 + W_{body}^2)$
* $I_{m,cm} = \frac{1}{8} M_{motor} D_{motor}^2$
* $I_{w,cm} = \frac{1}{8} M_{wheel} (D_{wheel\_out}^2 + D_{wheel\_in}^2)$
* **Moment of inertia of the robot assembly about the pivot O:**
    $I_{assembly,O} = (I_{b,cm} + M_{body}L_b^2) + (I_{m,cm} + M_{motor}L_m^2) + M_{wheel}L_m^2$
* **Effective mass-length product for gravitational torque:**
    $M_{eff}L_{eff} = M_{body}L_b + (M_{motor}+M_{wheel})L_m$

The equation can be written to explicitly show $\ddot{\theta}$:

$$\ddot{\theta}(t) = \frac{-M_{eff}L_{eff}g\sin\theta(t) - I_{w,cm}\ddot{\phi}(t)}{I_{assembly,O} + I_{w,cm}}$$

This is the desired mathematical model. To simulate or analyze this system, you can express it in state-space form. Let $x_1 = \theta$ and $x_2 = \dot{\theta}$. Let $u_{in}(t) = \ddot{\phi}(t)$.

$\dot{x_1}(t) = x_2(t)$
$\dot{x_2}(t) = \frac{-M_{eff}L_{eff}g\sin(x_1(t)) - I_{w,cm}u_{in}(t)}{I_{assembly,O} + I_{w,cm}}$

This model describes how the robot's tilt angle $\theta$ changes in response to the reaction wheel's relative acceleration $\ddot{\phi}$.
