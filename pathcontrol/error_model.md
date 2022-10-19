# Error Model


For the control of a robot it is more appropriate with most model based controllers ĺike LQR to use the relative pose between the robot and the path.
For this purpose the current pose of the robot $\mathbf p_c=\begin{bmatrix} x_c & y_c & \theta_c \end{bmatrix}^T$
is transformed into the coordinate system of the path reference pose $\mathbf p_r=\begin{bmatrix} x_r & y_r & \theta_r \end{bmatrix}^T$, resulting in the error pose $\mathbf p_e=\begin{bmatrix} x_e & y_e & \theta_e \end{bmatrix}^T$:

$\mathbf p_e  =  \begin{bmatrix} \cos{\theta_r} & \sin{\theta_r} & 0 \newline 
                                    -\sin{\theta_r} & \cos{\theta_r} & 0 \newline
                                     0 & 0 & 1
   						 \end{bmatrix}		 \begin{bmatrix}  x_c - x_r \newline  y_c - y_r \newline  \theta_c - \theta_r	 \end{bmatrix}$
