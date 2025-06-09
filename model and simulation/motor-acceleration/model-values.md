$$
\begin{gather*}
I_{w,cm}
= \frac{1}{2}\,M_{wheel}\bigl(R_{wheel_{out}}^2 + R_{wheel_{in}}^2\bigr)
= \frac{1}{2}\cdot0.109\,(0.63^2 + 0.52^2)
\approx 3.64\times10^{-2}\ \mathrm{kg\cdot m^2}.\\
K_D = 1.53\ \mathrm{\frac{rad/s^2}{dc}}
K_\omega = 1.14\ \mathrm{\frac{rad/s}{dc}}
\end{gather*}
$$

$$
\begin{aligned}
\ddot{\phi}(t) &= \frac{K_D}{I_{w,cm}} D(t) - \frac{K_\omega}{I_{w,cm}} \dot{\phi}(t) - \ddot{\theta}(t)\\
&= \frac{1.53\;\mathrm{rad/s^2/dc}}{3.64\times10^{-2}\;\mathrm{kg\cdot m^2}}\;D(t)
\;-\;\frac{1.14\;\mathrm{rad/s/dc}}{3.64\times10^{-2}\;\mathrm{kg\cdot m^2}}\;\dot{\phi}(t)
\;-\;\ddot{\theta}(t)\\
&\approx 42.0\;\mathrm{\frac{rad}{s^2\,dc}}\,D(t)
\;-\;31.3\;\mathrm{\frac{1}{s\,dc}}\,\dot{\phi}(t)
\;-\;\ddot{\theta}(t).
\end{aligned}
$$

$$
\boxed{\ddot{\phi}(t)\approx 42.0\,D(t)\;-\;31.3\,\dot{\phi}(t)\;-\;\ddot{\theta}(t).}
$$
