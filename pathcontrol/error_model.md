# Error Model


For the control of a robot it is more appropriate with most model based controllers like LQR to use the relative pose between the robot and the path.
For this purpose the current pose of the robot $\mathbf p_c=\begin{bmatrix} x_c & y_c & \theta_c \end{bmatrix}^T$
is transformed into the coordinate system of the path reference pose $\mathbf p_r=\begin{bmatrix} x_r & y_r & \theta_r \end{bmatrix}^T$, resulting in the error pose $\mathbf p_e=\begin{bmatrix} x_e & y_e & \theta_e \end{bmatrix}^T$:

$\mathbf p_e  =  \begin{bmatrix} \cos{\theta_r} & \sin{\theta_r} & 0 \newline 
                                    -\sin{\theta_r} & \cos{\theta_r} & 0 \newline
                                     0 & 0 & 1
   						 \end{bmatrix}		 \begin{bmatrix}  x_c - x_r \newline  y_c - y_r \newline  \theta_c - \theta_r	 \end{bmatrix}$.

The error dynamics $\mathbf{\dot p_e} = \begin{bmatrix} \dot x_e & \dot y_e & \dot \theta_e\end{bmatrix}^T$ is calculated by the time derivative of $\mathbf p_e$.
Using a more general kinematic model 
$\mathbf x = \begin{bmatrix}  \dot x_0 \newline \dot y_o \newline \dot \theta \end{bmatrix}
           =  \begin{bmatrix} v \cos \theta - \dot \theta l_o \sin \theta \newline 
                              v \cos \theta - \dot \theta l_o \sin \theta \newline
                              \frac{v}{l_w} \tan \delta_f \end{bmatrix}$
with an offset $l_o$ in the longitudinal axis the following error model is calculated:
$\dot p_e   =   \begin{bmatrix}          y_e - v_r + v_c \cos{\theta_e} - l_o \omega_c \sin{\theta_e} \newline
        -\omega_r x_e - l_o w_r + v_c \sin{\theta_e} + l_o \omega_c \cos{\theta_e} \newline
        \omega_c - \omega_r  \omega_r    \end{bmatrix}$.
This error model can be further simplified by assuming that the reference pose velocities $v_r$ and $\omega_r$ are disturbances which are not present ($v_r= 0$  , $\omega_r= 0$).

