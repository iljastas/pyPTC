# Feed forward for Ackerman-steered (car-like) vehicles

The path (its x- and y-positions) is time indepedent, but a path tracking control and its feed forward are time depedent.
A path can be described parametric from the path length <img src="https://latex.codecogs.com/svg.image?s"/>. But the time deviation of the path length <img src="https://latex.codecogs.com/svg.image?s"/> is the driving velocity of the vehicle:
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\dot&space;s&space;=&space;\frac{ds}{dt}&space;=&space;v(t)" title="\dot s = \frac{ds}{dt} = v(t)" />.
</p>

Here, flatness-based feedforward controls are derived, because for nonlinear systems it is more accurate.

For a pure kinematic model
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\mathbf{\dot x}=&space;\begin{bmatrix}&space;x^\prime(s)&space;\\&space;y^\prime(s)&space;&space;\\&space;\theta^\prime(s)\end{bmatrix}=&space;\begin{bmatrix}&space;v(s)&space;\cos(\theta(s))&space;\\&space;v(s)&space;\cos(\theta(s))&space;&space;\\&space;&space;\frac{v(s)}{l_r}\tan(\delta(s))\end{bmatrix}&space;&space;" title="" />
</p>
the flat output is:
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

The steering angle <img src="https://latex.codecogs.com/svg.image?\delta" title="\delta" /> can be calculated easly:
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\delta(t)&space;=&space;\arctan\biggl(\frac{l_r&space;\dot&space;\theta}{v(t)}\biggl)" title="\delta(s) = \arctan\biggl(\frac{l_r \theta^\prime(s)}{v(s)}\biggl)" />.
</p>

But transform <img src="https://latex.codecogs.com/svg.image?\theta^\prime(s)" title="\theta^\prime(s)" /> into the time, the coherence
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\dot&space;\theta&space;=&space;\frac{d&space;\theta}{dt}&space;=&space;\frac{d&space;\theta}{dt}&space;\frac{ds}{ds}&space;=&space;&space;\frac{d&space;\theta}{ds}&space;\frac{ds}{dt}&space;=&space;\theta^\prime(s)&space;\dot&space;s" title="\dot \theta = \frac{d \theta}{dt} = \frac{d \theta}{dt} \frac{ds}{ds} = \frac{d \theta}{ds} \frac{ds}{dt} = \theta^\prime(s) \dot s" />
</p>
and
<p align="center">
<img src="https://latex.codecogs.com/svg.image?&space;v(s)&space;=&space;\frac{v(t)}{\dot&space;s}&space;=&space;\frac{1}{\dot&space;s}&space;\sqrt{\dot&space;x^2&space;&plus;&space;\dot&space;y^2}&space;=&space;\sqrt{x^{\prime2}(s)&space;&plus;&space;y^{\prime2}(s)&space;}" title=" v(s) = \frac{v(t)}{\dot s} = \frac{1}{\dot s} \sqrt{\dot x^2 + \dot y^2} = \sqrt{x^{\prime2}(s) + y^{\prime2}(s) }" />
</p>
is used to
<p align="center">
<img src="https://latex.codecogs.com/svg.image?\delta(t)&space;=&space;\arctan{\biggl(&space;l_w&space;\frac{\theta^\prime(s)&space;\dot&space;s(t)&space;&space;}{v(s)&space;\dot&space;s(t)&space;&space;}&space;\biggl)}&space;\textrm{sign}(v(t))=&space;\arctan{\biggl(&space;l_w&space;\frac{\theta^\prime(s)&space;}{v(s)&space;&space;}&space;\biggl)}\textrm{sign}(v(t))" title="\delta(t) = \arctan{\biggl( l_w \frac{\theta^\prime(s) \dot s(t) }{v(s) \dot s(t) } \biggl)} \textrm{sign}(v(t))= \arctan{\biggl( l_w \frac{\theta^\prime(s) }{v(s) } \biggl)}\textrm{sign}(v(t))" />
</p>

## References
[1] B. Müller, J. Deutscher and S. Grodde, "Continuous Curvature Trajectory Design and Feedforward Control for Parking a Car," in IEEE Transactions on Control Systems Technology, vol. 15, no. 3, pp. 541-553, May 2007, doi: 10.1109/TCST.2006.890289.

[2] B. Müller, J. Deutscher and S. Grodde, "Trajectory generation and feedforward control for parking a car," 2006 IEEE Conference on Computer Aided Control System Design, 2006 IEEE International Conference on Control Applications, 2006 IEEE International Symposium on Intelligent Control, 2006, pp. 163-168, doi: 10.1109/CACSD-CCA-ISIC.2006.4776641.

[3] B. Müller and J. Deutscher, "Orbital tracking control for car parking via control of the clock using a nonlinear reduced order steering-angle observer," 2007 European Control Conference (ECC), 2007, pp. 1917-1924, doi: 10.23919/ECC.2007.7068555.
