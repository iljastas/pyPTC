# Feed forward for Ackerman-steered (car-like) vehicles

The path (its x- and y-positions) is time indepedent, but a path tracking control and its feed forward are time depedent.
A path can be described parametric from the path length <img src="https://latex.codecogs.com/svg.image?s"/>.

Here, flatness-based feedforward controls are derived, because for nonlinear systems it is more accurate.

The flat output is:
<p align="center">
<img src="https://latex.codecogs.com/svg.image?y_f(s)&space;=&space;\begin{bmatrix}&space;x(s)&space;&&space;y(s)&space;&space;\\\end{bmatrix}^T" title="" />.
</p>

The velocity can be formulated from the flat output and its derivation:
<p align="center">
<img src="https://latex.codecogs.com/svg.image?v(s)&space;=&space;\sqrt{x^{\prime&space;2}(s)&space;&plus;&space;y^{\prime&space;2}(s)&space;}" title="" />.
</p>

The orientation (yaw angle) can be formulated from the flat output and its derivation:
<p align="center">
<img align="center" src="https://latex.codecogs.com/svg.image?\theta(s)=\arctan\biggl({\frac{y^\prime(s)}{x^\prime(s)}}\biggl)" title="\theta(s)=\arctan\biggl({\frac{y^\prime(s)}{x^\prime(s)}}\biggl)" />.
</p>

The yaw rate (deviation of the orientation) can also be formulated from the flat output and its derivation:
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\theta^\prime(s)&space;=&space;\frac{x^{\prime\prime}y^{\prime}&space;-&space;x^{\prime}&space;y^{\prime\prime}&space;}{x^{\prime&space;2}&plus;y^{\prime&space;2}}" title="\theta^\prime(s) = \frac{x^{\prime\prime}y^{\prime} - x^{\prime} y^{\prime\prime} }{x^{\prime 2}+y^{\prime 2}}" />
</p>

## Kinematic feed forward

#


## References
[1] B. Muller, J. Deutscher and S. Grodde, "Continuous Curvature Trajectory Design and Feedforward Control for Parking a Car," in IEEE Transactions on Control Systems Technology, vol. 15, no. 3, pp. 541-553, May 2007, doi: 10.1109/TCST.2006.890289.

[2] B. Muller, J. Deutscher and S. Grodde, "Trajectory generation and feedforward control for parking a car," 2006 IEEE Conference on Computer Aided Control System Design, 2006 IEEE International Conference on Control Applications, 2006 IEEE International Symposium on Intelligent Control, 2006, pp. 163-168, doi: 10.1109/CACSD-CCA-ISIC.2006.4776641.

[3] B. Müller and J. Deutscher, "Orbital tracking control for car parking via control of the clock using a nonlinear reduced order steering-angle observer," 2007 European Control Conference (ECC), 2007, pp. 1917-1924, doi: 10.23919/ECC.2007.7068555.
