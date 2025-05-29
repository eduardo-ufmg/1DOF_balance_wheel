Okay, let's derive the mathematical model for your self-balancing robot.

The core idea is to use Newton's second law for rotation: the sum of torques about the pivot point equals the total moment of inertia (about the pivot) times the angular acceleration of the robot body. We also need to consider the reaction torque from the accelerating wheel.

Let `θ` be the angle of the robot's longitudinal axis with respect to the vertical. A positive `θ` could represent a tilt to the right.
Let `α = d²θ/dt²` be the angular acceleration of the robot body.
Let `α_w_rel` be the angular acceleration of the motor's shaft (and thus the wheel) *relative* to the robot's body. This is your system **input**.

The equation of motion for the robot body's tilt is:
$$(I_{sys\_pivot} + I_{wheel\_shaft}) \frac{d^2\theta}{dt^2} = M_{eff\_dist} \cdot g \cdot \sin(\theta) - I_{wheel\_shaft} \cdot \alpha_{w\_rel}$$

This can be rewritten to directly show the output ($d^2\theta/dt^2$) as a function of state ($\theta$) and input ($\alpha_{w\_rel}$):
$$\frac{d^2\theta}{dt^2} = \frac{M_{eff\_dist} \cdot g \cdot \sin(\theta) - I_{wheel\_shaft} \cdot \alpha_{w\_rel}}{I_{sys\_pivot} + I_{wheel\_shaft}}$$

Here's a breakdown of the terms:

---
## Variables

* **Output Variable**:
    * `θ`: Angle of the robot's longitudinal axis with the plane vertical axis (radians).
    * `dθ/dt`: Angular velocity of the robot body (rad/s).
    * `d²θ/dt²`: Angular acceleration of the robot body (rad/s²). This is the direct output of the EOM.
* **Input Variable**:
    * `α_w_rel`: Angular acceleration of the motor's shaft (and wheel) relative to the robot body (rad/s²).
* **Constant**:
    * `g`: Acceleration due to gravity (approx. 9.81 m/s²).

---
## System Parameters (from your description)

* `M_body`: Mass of the robot's body (green rectangle).
* `W_body`: Width of the robot's body (e.g., 54 mm from the image, convert to meters).
* `H_body`: Height of the robot's body.
* `L_stem`: Length of the massless stem connecting the body to the balance point.
* `M_wheel`: Mass of the reaction wheel (blue ring).
* `D_wheel_out`: Outer diameter of the reaction wheel.
* `D_wheel_in`: Inner diameter of the reaction wheel.
* `M_motor`: Mass of the motor (red circle).
* `D_motor`: Diameter of the motor.

---
## Calculated Terms

1.  **Distances from Pivot Point to Centers of Mass (CM):**
    * We assume the pivot point is where the stem meets the ground/support.
    * CM of Body: `L_c_body = L_stem + H_body/2`
    * CM of Motor & Wheel: Assuming the motor is mounted on top of the body, and its center (and thus the wheel's axis) is `D_motor/2` above the top surface of the body:
        `L_c_motor_wheel = L_stem + H_body + D_motor/2`

2.  **Moments of Inertia (MoI):**
    * **Body about its own CM (`I_body_cm`):** For a rectangular plate rotating about an axis perpendicular to its face:
        `I_body_cm = (1/12) * M_body * (H_body² + W_body²) `
    * **Body about the pivot point (`I_body_pivot`):** Using the Parallel Axis Theorem:
        `I_body_pivot = I_body_cm + M_body * L_c_body²`
    * **Motor about the pivot point (`I_motor_pivot`):** Treating the motor as a point mass for the body's rotation:
        `I_motor_pivot = M_motor * L_c_motor_wheel²`
    * **Wheel's contribution to body's rotation about pivot (`I_wheel_body_pivot`):** Treating the wheel as a point mass for the body's rotation:
        `I_wheel_body_pivot = M_wheel * L_c_motor_wheel²`
    * **Total system MoI about the pivot (`I_sys_pivot`):** This is the MoI of the entire assembly (body + motor + wheel) if it were a single rigid object rotating about the pivot.
        `I_sys_pivot = I_body_pivot + I_motor_pivot + I_wheel_body_pivot`
    * **Wheel about its own shaft (`I_wheel_shaft`):** For a ring or hollow cylinder:
        `R_wheel_out = D_wheel_out / 2`
        `R_wheel_in = D_wheel_in / 2`
        `I_wheel_shaft = (1/2) * M_wheel * (R_wheel_out² + R_wheel_in²) `

3.  **Effective Mass-Distance for Gravitational Torque (`M_eff_dist`):**
    This term collects all parts contributing to the gravitational torque that tries to tip the robot over.
    `M_eff_dist = (M_body * L_c_body) + (M_motor * L_c_motor_wheel) + (M_wheel * L_c_motor_wheel)`

---
## Explanation of the Equation of Motion

* `(I_sys_pivot + I_wheel_shaft) \frac{d^2\theta}{dt^2}`: This is the "inertial torque" term. `I_sys_pivot` is the inertia of the whole robot structure resisting the tilt `d²θ/dt²`. The `I_wheel_shaft` term appears here because the absolute acceleration of the wheel is `d²θ/dt² + α_w_rel`. The torque required to produce this absolute acceleration of the wheel results in an equal and opposite torque on the body, effectively increasing its resistance to acceleration when the wheel itself is being accelerated.
* `M_eff_dist \cdot g \cdot \sin(\theta)`: This is the **gravitational torque** acting to destabilize the robot (pull it down) when it's tilted by an angle `θ`.
* `- I_{wheel\_shaft} \cdot \alpha_{w\_rel}`: This is the **reaction torque** from the wheel onto the robot body. When the motor applies a torque to accelerate the wheel with `α_w_rel` (relative to the body), the wheel exerts an equal and opposite torque on the motor, and thus on the robot body. This is the control torque used for balancing.

---
## Linearized Model (for small angles)

For control design, it's often useful to linearize the model around the upright position (`θ ≈ 0`). In this case, `sin(θ) ≈ θ`.
The linearized equation of motion becomes:
$$(I_{sys\_pivot} + I_{wheel\_shaft}) \frac{d^2\theta}{dt^2} = M_{eff\_dist} \cdot g \cdot \theta - I_{wheel\_shaft} \cdot \alpha_{w\_rel}$$
$$\frac{d^2\theta}{dt^2} = \frac{M_{eff\_dist} \cdot g \cdot \theta - I_{wheel\_shaft} \cdot \alpha_{w\_rel}}{I_{sys\_pivot} + I_{wheel\_shaft}}$$
This is a linear second-order ordinary differential equation.

---
## Key Assumptions Made

1.  The stem and wheel spokes are massless.
2.  The robot only tilts in the plane shown (single degree of freedom for tilt).
3.  The motor's center (and thus the wheel's axis) is located at a distance `D_motor/2` above the top surface of the robot's body, along its longitudinal centerline.
4.  The "motor's shaft angular acceleration" (`α_w_rel`) is the acceleration of the wheel *relative* to the robot body, and this is the control input.
5.  Friction at the pivot and air resistance are negligible.

This mathematical model provides the relationship between the motor's relative angular acceleration and the resulting angular acceleration of the robot's body, forming the basis for designing a control system to keep it balanced.