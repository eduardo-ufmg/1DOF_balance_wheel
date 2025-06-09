$$
\begin{aligned}
M_{body} &= 48\ \mathrm{g} = 4.8\mathrm{e-2}\ \mathrm{kg}\\
W_{body} &= 54\ \mathrm{cm} = 5.4\mathrm{e-1}\ \mathrm{m}\\
H_{body} &= 95\ \mathrm{cm} = 9.5\mathrm{e-1}\ \mathrm{m}\\
L_{stem} &= 44\ \mathrm{cm} = 4.4\mathrm{e-1}\ \mathrm{m}\\
M_{wheel} &= 109\ \mathrm{g} = 1.09\mathrm{e-1}\ \mathrm{kg}\\
D_{wheel_{out}} &= 126\ \mathrm{cm} = 1.26\mathrm{m}\\
D_{wheel_{in}} &= 104\ \mathrm{cm} = 1.04\mathrm{m}\\
M_{motor} &= 112\ \mathrm{g} = 1.12\mathrm{e-1}\ \mathrm{kg}\\
D_{motor} &= 42\ \mathrm{cm} = 4.2\mathrm{e-1}\ \mathrm{m}\\
g &= 9.81\ \mathrm{m/s}^2\\
\end{aligned}
$$

$$
\begin{aligned}
R_{motor} &= \frac{D_{motor}}{2}
    = \frac{4.2\times10^{-1}}{2}
    = 2.1\times10^{-1}\ \mathrm{m},\\
R_{wheel_{out}} &= \frac{D_{wheel_{out}}}{2}
    = \frac{1.26}{2}
    = 6.3\times10^{-1}\ \mathrm{m},\\
R_{wheel_{in}} &= \frac{D_{wheel_{in}}}{2}
    = \frac{1.04}{2}
    = 5.2\times10^{-1}\ \mathrm{m},\\
L_b &= L_{stem} + \frac{H_{body}}{2}
    = 4.4\times10^{-1} + \frac{9.5\times10^{-1}}{2}
    = 9.15\times10^{-1}\ \mathrm{m},\\
L_m &= L_{stem} + H_{body} + R_{motor}
    = 4.4\times10^{-1} + 9.5\times10^{-1} + 2.1\times10^{-1}
    = 1.60\times10^{0}\ \mathrm{m}.
\end{aligned}
$$

$$
\begin{aligned}
I_{b,cm}
&= \frac{1}{12}\,M_{body}\bigl(H_{body}^2 + W_{body}^2\bigr)
= \frac{1}{12}\cdot0.048\,(0.95^2 + 0.54^2)
\approx 4.78\times10^{-3}\ \mathrm{kg\cdot m^2},\\
I_{m,cm}
&= \frac{1}{2}\,M_{motor}\,R_{motor}^2
= \frac{1}{2}\cdot0.112\,(0.21^2)
\approx 2.47\times10^{-3}\ \mathrm{kg\cdot m^2},\\
I_{w,cm}
&= \frac{1}{2}\,M_{wheel}\bigl(R_{wheel_{out}}^2 + R_{wheel_{in}}^2\bigr)
= \frac{1}{2}\cdot0.109\,(0.63^2 + 0.52^2)
\approx 3.64\times10^{-2}\ \mathrm{kg\cdot m^2}.
\end{aligned}
$$

$$
\begin{aligned}
I_{assembly,O}
&= \bigl(I_{b,cm} + M_{body}L_b^2\bigr) + \bigl(I_{m,cm} + M_{motor}L_m^2\bigr) + M_{wheel}\,L_m^2\\
&\approx \bigl(4.78\times10^{-3} + 0.048\,(0.915^2)\bigr) + \bigl(2.47\times10^{-3} + 0.112\,(1.60^2)\bigr) + 0.109\,(1.60^2)\\
&\approx 6.13\times10^{-1}\ \mathrm{kg\cdot m^2},
\end{aligned}
$$

$$
\begin{aligned}
M_{eff}L_{eff}
&= M_{body}\,L_b + (M_{motor}+M_{wheel})\,L_m
= 0.048\cdot0.915 + (0.112+0.109)\cdot1.60
\approx 3.98\times10^{-1}\ \mathrm{kg\cdot m}.
\end{aligned}
$$

$$
\ddot{\theta}(t)
= \frac{-M_{eff}L_{eff}\,g\;\sin\!\bigl(\theta(t)\bigr)\;-\,I_{w,cm}\,\ddot{\phi}(t)}
       {\,I_{assembly,O} + I_{w,cm}\,}
= \frac{-\bigl(0.3978\cdot9.81\bigr)\sin\theta(t)\;-\;(0.0364)\,\ddot\phi(t)}
       {0.6132 + 0.0364},
$$

$$
\boxed{
\ddot{\theta}(t)
= \frac{-3.900\,\sin\!\bigl(\theta(t)\bigr)\;-\;0.0364\,\ddot{\phi}(t)}
       {0.6496}
}.
$$

