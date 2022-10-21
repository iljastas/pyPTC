# LQR with Feedback Linearization

[Using the error model](../error_model.md)

## Ackermann
The simplest LQR control for the Ackermann steering can be designed with the purely kinematic error model and using steering angle $u=\delta$ as input and the velocity at the rear axle $v_h$.

The derivation can be find in this [file](https://duckduckgo.com).

\begin{align} 
    \dot{\mathbf x}
    =
    \begin{bmatrix}
        \dot y_e \\ \dot \theta_e
    \end{bmatrix}
    &=
    \begin{bmatrix}
        v_{rx} \frac{\sin(\theta_e + \alpha_r)}{\cos\alpha_r} + \frac{v_{rx}}{l_w} l_o  \cos(\theta_e + \alpha_r)  \bigl( \tan(u + \alpha_f ) - \tan\alpha_r \bigl)  \\
        \frac{v_{rx}}{l_w}  \bigl( \tan(u + \alpha_f ) - \tan\alpha_r \bigl)
    \end{bmatrix}
       \\ & \stackrel{\tilde u =  \tan(u + \alpha_f) }{=} 
    \underbrace{
        \begin{bmatrix}
            v_{rx} \frac{\sin(\theta_e + \alpha_r)}{\cos\alpha_r} -
            \frac{v_{rx}}{l_w} l_o \cos(\theta_e + \alpha_r) \,  {\tan}\alpha_r \\
             -\frac{v_{rx} }{l_w} \tan \alpha_r
        \end{bmatrix}
    }_{ \mathbf a(\mathbf x) }
    +
    \underbrace{
        \begin{bmatrix}
          \frac{ v_{rx}}{l_w} l_o \cos(\theta_e + \alpha_r)
           \\
           \frac{v_{{hx}}}{l_w}
        \end{bmatrix}
    }_{ \mathbf b(\mathbf x) }
    \tilde u
    \label{eq_chap5__fbl_lqr__ackr_errmodel_alpha}
\end{align}
