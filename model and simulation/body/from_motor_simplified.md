
$$
x = \begin{bmatrix}\theta \\ \dot\theta \\ \phi \\ \dot\phi\end{bmatrix}
\qquad
u = \begin{bmatrix} d \end{bmatrix}
$$

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0\\[6pt]
\frac{M_{\rm tot}g\,d}{I} & 0 & 0 & \frac{k_mk_e}{R\,I}\\[6pt]
0 & 0 & 0 & 1\\[6pt]
-\frac{M_{\rm tot}g\,d}{I} & 0 & 0 & -\frac{k_mk_e}{R}\frac{I+I_{\rm wheel}}{I\,I_{\rm wheel}}
\end{bmatrix}
\qquad
B = \begin{bmatrix}
0\\[6pt]
-\frac{12k_m}{R\,I}\\[6pt]
0\\[6pt]
\frac{12k_m}{R}\frac{I+I_{\rm wheel}}{I\,I_{\rm wheel}}
\end{bmatrix}
$$

$d$ is the PWM duty cycle, $\in [-1,1]$.
