## Experimental Parameter Identification

To find $K_D$ and $K_{\omega}$, we need to conduct experiments. Ideally, these experiments should isolate the motor-wheel dynamics as much as possible by constraining the robot body's motion (i.e., making $\theta, \dot{\theta}, \ddot{\theta}$ all zero).

**Setup for Experiments:**
Securely fix the robot body so it cannot tilt ($\theta(t)=0 \implies \dot{\theta}(t)=0, \ddot{\theta}(t)=0$).
In this fixed setup, the model simplifies to:
$\ddot{\phi}(t) = \frac{K_D}{I_{w,cm}} D(t) - \frac{K_{\omega}}{I_{w,cm}} \dot{\phi}(t)$

**Experiment 1: Stall Torque Characteristics (to find $K_D$)**
This aims to find the relationship between duty cycle and torque at or near zero speed.
1.  With the robot body fixed and the wheel initially at rest ($\dot{\phi} \approx 0$).
2.  Apply a small, constant duty cycle $D$.
3.  Measure the initial angular acceleration $\ddot{\phi}_{init}$ of the wheel immediately after applying the PWM signal (before $\dot{\phi}$ becomes significant). This requires high-frequency sampling of the encoder data and numerical differentiation.
4.  Under these conditions ($\dot{\phi} \approx 0, \ddot{\theta}=0$):
    $\ddot{\phi}_{init} \approx \frac{K_D}{I_{w,cm}} D$
5.  Repeat for several different values of $D$.
6.  Plot $\ddot{\phi}_{init}$ (y-axis) versus $D$ (x-axis). This should yield a straight line.
7.  The slope of this line is $m_1 = K_D/I_{w,cm}$.
8.  Since $I_{w,cm}$ is known, calculate $K_D = m_1 \cdot I_{w,cm}$.

**Experiment 2: Steady-State Speed Characteristics (to find $K_{\omega}$)**
This aims to find the relationship between duty cycle and speed when the wheel is not accelerating.
1.  With the robot body fixed ($\ddot{\theta}=0$).
2.  Apply a constant duty cycle $D$.
3.  Allow the wheel to reach a steady-state angular velocity $\dot{\phi}_{ss}$. At steady state, $\ddot{\phi}=0$.
4.  Under these conditions ($\ddot{\phi}=0, \ddot{\theta}=0$):
    $0 = \frac{K_D}{I_{w,cm}} D - \frac{K_{\omega}}{I_{w,cm}} \dot{\phi}_{ss}$
    $K_D D = K_{\omega} \dot{\phi}_{ss}$
    $\dot{\phi}_{ss} = \frac{K_D}{K_{\omega}} D$
5.  Repeat for several different values of $D$, recording the corresponding $\dot{\phi}_{ss}$.
6.  Plot $\dot{\phi}_{ss}$ (y-axis) versus $D$ (x-axis). This should also yield a straight line.
7.  The slope of this line is $m_2 = K_D/K_{\omega}$.
8.  Since $K_D$ is known from Experiment 1, calculate $K_{\omega} = K_D / m_2$.
    Alternatively, plot $K_D D$ (y-axis, using $K_D$ from Exp.1) vs $\dot{\phi}_{ss}$ (x-axis). The slope is directly $K_{\omega}$.