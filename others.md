But in reality, it is much simpler to calulate the Jacobian relative to the the current frame $T_6$ than it is to calculate relative to the first frame. Therefore, we will instead use the following approach. 

$$[^{T_6}D]=[^{T_6}J][D_\theta]$$

We can calculate the Jacobian with respect to the last frame using the following formula.

- The differential motion relationship can be written as:
$$\left[\begin{matrix}
    ^{T_6}dx\\^{T_6}dy\\^{T_6}dz\\^{T_6}\delta x\\^{T_6}\delta y\\^{T_6}\delta z\end{matrix}
\right]
=
\left[\begin{matrix}
^{T_6}J_{11} & ^{T_6}J_{12} & . & . & . & ^{T_6}J_{16}\\
^{T_6}J_{21} & ^{T_6}J_{22} & . & . & . & ^{T_6}J_{26}\\
^{T_6}J_{31} & ^{T_6}J_{32} & . & . & . & ^{T_6}J_{36}\\
^{T_6}J_{41} & ^{T_6}J_{42} & . & . & . & ^{T_6}J_{46}\\
^{T_6}J_{51} & ^{T_6}J_{52} & . & . & . & ^{T_6}J_{56}\\
^{T_6}J_{61} & ^{T_6}J_{62} & . & . & . & ^{T_6}J_{66}
\end{matrix}\right]
\left[\begin{matrix}
    d\theta_1\\d\theta_2\\d\theta_3\\d\theta_4\\d\theta_5\\d\theta_6\end{matrix}
\right]$$
- Assuming that any combination of $A_1A_2...A_n$ can be expressed with a corresponding $n,o,a,p$ matrix, the corresponding elements of the matrix will be used to calculate the Jacobian.
- If joint $i$ under consideration is a revolute joint, then:
$$\begin{array}{ccc}
    ^{T_6}J_{1i}=(-n_xp_y+n_yp_x) & ^{T_6}J_{2i}=(-o_xp_y+o_yp_x) & ^{T_6}J_{3i}=(-a_xp_y+a_yp_x) \\
    ^{T_6}J_{4i}=n_z & ^{T_6}J_{5i}=o_z & ^{T_6}J_{6i}=a_z
\end{array}$$
- If joint $i$ under consideration is a prismatic joint, then:
$$\begin{array}{ccc}
    ^{T_6}J_{1i}=n_z & ^{T_6}J_{2i}=o_z & ^{T_6}J_{3i}=a_z \\
    ^{T_6}J_{4i}=0 & ^{T_6}J_{5i}=0 & ^{T_6}J_{6i}=0
\end{array}$$
- For Equation (1) and (2), for column $i$ use $^{i-1}T_6$, meaning:\
For column 1, use $^0T_6=A_1A_2A_3A_4A_5A_6$\
For column 2, use $^1T_6=A_2A_3A_4A_5A_6$\
For column 3, use $^2T_6=A_3A_4A_5A_6$\
For column 4, use $^3T_6=A_4A_5A_6$\
For column 5, use $^4T_6=A_5A_6$\
For column 6, use $^5T_6=A_6$

> The following part is used to derive the Jacobian relative to the the current frame. It's useless in the control program. And I am not sure the result is exactly correct.

$$A_1 = \left[
\begin{matrix}
    cos(\theta_1) & 0 & sin(\theta_1) & 0\\
    sin(\theta_1) & 0 & -cos(\theta_1) & 0\\
    0 & 1 & 0 & 0\\
    0 & 0 & 0 & 1
\end{matrix}
\right]$$

$$A_2 = \left[
\begin{matrix}
    cos(\theta_2) & -sin(\theta_2) & 0 & a_1cos(\theta_2)\\
    sin(\theta_2) & cos(\theta_2) & 0 & a_1sin(\theta_2)\\
    0 & 0 & 1 & d\\
    0 & 0 & 0 & 1
\end{matrix}
\right]$$

$$A_3 = \left[
\begin{matrix}
    cos(\theta_3) & -sin(\theta_3) & 0 & a_2cos(\theta_3)\\
    sin(\theta_3) & cos(\theta_3) & 0 & a_2sin(\theta_3)\\
    0 & 0 & 1 & 0\\
    0 & 0 & 0 & 1
\end{matrix}
\right]$$

For column 1, use $^0T_3=A_1A_2A_3$


$$^0T_3 = A_1A_2A_3 = 
\left[
\begin{matrix}
    C_1C_{23}& - C_1S_{23}&  S_1&       dS_1 + a_1C_1C_2 + a_2C_1C_{23}\\
    S_1C_{23}& - S_1S_{23}& -C_1&       -dC_1 + a_1S_1C_2 + a_2S_1C_{23}\\
       S_{23}&      C_{23}&    0&                a_1S_2 + a_2S_{23}\\
            0&           0&    0&                                 1
\end{matrix}
\right]
=
\left[
    \begin{matrix}
        n_x & o_x & a_x & p_x\\
        n_y & o_y & a_y & p_y\\
        n_z & o_z & a_z & p_z\\
        0 & 0 & 0 & 1
    \end{matrix}
    \right]
$$

$$\begin{aligned}
    ^{T_3}J_{11}&=-n_xp_y+n_yp_x\\
    &=-C_1C_{23}\cdot(-dC_1 + a_1S_1C_2 + a_2S_1C_{23})+S_1C_{23}\cdot(dS_1 + a_1C_1C_2 + a_2C_1C_{23}) \\
    &=dC_{23}\\
    ^{T_3}J_{21}&=-o_xp_y+o_yp_x\\
    &=C_1S_{23}\cdot(-dC_1 + a_1S_1C_2 + a_2S_1C_{23})- S_1S_{23}\cdot(dS_1 + a_1C_1C_2 + a_2C_1C_{23}) \\
    &=-dS_{23}\\
    ^{T_3}J_{31}&=-a_xp_y+a_yp_x\\
    &=-S_1\cdot(-dC_1 + a_1S_1C_2 + a_2S_1C_{23})-C_1\cdot(dS_1 + a_1C_1C_2 + a_2C_1C_{23})\\
    &=-a_1C_2-a_2C_{23}
\end{aligned}$$

For column 2, use $^1T_3=A_2A_3$
$$^1T_3 = A_2A_3 = 
\left[
\begin{matrix}
    C_{23}& -S_{23}&  0& a_1C_2 + a_2C_{23}\\
    S_{23}&  C_{23}&  0& a_1S_2 + a_2S_{23}\\
         0&       0&  1&                  d\\
         0&       0&  0&                  1
\end{matrix}
\right]
=
\left[
    \begin{matrix}
        n_x & o_x & a_x & p_x\\
        n_y & o_y & a_y & p_y\\
        n_z & o_z & a_z & p_z\\
        0 & 0 & 0 & 1
    \end{matrix}
    \right]
$$

$$\begin{aligned}
    ^{T_3}J_{12}&=-n_xp_y+n_yp_x\\
    &=-C_{23}\cdot(a_1S_2 + a_2S_{23})+S_{23}\cdot(a_1C_2 + a_2C_{23}) \\
    &=a_1S_3\\
    ^{T_3}J_{22}&=-o_xp_y+o_yp_x\\
    &=S_{23}\cdot(a_1S_2 + a_2S_{23})+C_{23}\cdot(a_1C_2 + a_2C_{23}) \\
    &=a_1C_3+a_2\\
    ^{T_3}J_{32}&=-a_xp_y+a_yp_x\\
    &=0
\end{aligned}$$

For column 3, use $^2T_3=A_3$
$$^2T_3 = A_3 = \left[
\begin{matrix}
    C_3 & -S_3 & 0 & a_2C_3\\
    S_3 & C_3 & 0 & a_2S_3\\
    0 & 0 & 1 & 0\\
    0 & 0 & 0 & 1
\end{matrix}
\right]
=
\left[
    \begin{matrix}
        n_x & o_x & a_x & p_x\\
        n_y & o_y & a_y & p_y\\
        n_z & o_z & a_z & p_z\\
        0 & 0 & 0 & 1
    \end{matrix}
    \right]
$$
$$\begin{aligned}
    ^{T_3}J_{13}&=-n_xp_y+n_yp_x\\
    &=-C_{3}\cdot a_2S_3+S_3\cdot a_2C_3 \\
    &=0\\
    ^{T_3}J_{23}&=-o_xp_y+o_yp_x\\
    &=S_3\cdot a_2S_3+C_3\cdot a_2C_3 \\
    &=a_2\\
    ^{T_3}J_{33}&=-a_xp_y+a_yp_x\\
    &=0
\end{aligned}$$

Therefore,
$$^{T_3}J=
\left[\begin{matrix}
        dC_{23} & a_1S_3 & 0\\
        -dS_{23} & a_1C_3+a_2 & a_2\\
        -a_1C_2-a_2C_{23} & 0 & 0
\end{matrix}\right]$$