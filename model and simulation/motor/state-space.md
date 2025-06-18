Let the state, input, and output vectors be:

$$
\mathbf{x} = \begin{bmatrix} i_a \\ \omega_m \end{bmatrix}, \quad
\mathbf{u} = \begin{bmatrix} V_{\text{avg}} \\ T_l \end{bmatrix}, \quad
\mathbf{y} = \begin{bmatrix} \omega_m \end{bmatrix}
$$

The state-space representation is:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

$$
\mathbf{y} = C\mathbf{x} + D\mathbf{u}
$$

where

$$
A = \begin{bmatrix}
-\frac{R}{L} & -\frac{K_e}{L} \\
\frac{K_t}{J} & -\frac{B}{J}
\end{bmatrix}, \quad
B = \begin{bmatrix}
\frac{1}{L} & 0 \\
0 & -\frac{1}{J}
\end{bmatrix}
$$

$$
C = \begin{bmatrix} 0 & 1 \end{bmatrix}, \quad
D = \begin{bmatrix} 0 & 0 \end{bmatrix}
$$