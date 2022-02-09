# Feed forward for Ackerman-steered (car-like) vehicles

The path (its x- and y-positions) is time indepedent, but a path tracking control and its feed forward are time depedent.

Here, flatness-based feedforward controls are derived, because for nonlinear systems it is more accurate.

The flat ouput is
<img src="https://latex.codecogs.com/svg.image?y_f&space;=&space;\begin{bmatrix}&space;x&space;&&space;y&space;&space;\\\end{bmatrix}^T" title="y_f = \begin{bmatrix} x & y \\\end{bmatrix}^T" />.

The velocity <img src="https://latex.codecogs.com/svg.image?v&space;=&space;\sqrt{x^2(s)&space;&plus;&space;\dot{x}^2&space;}" title="v = \sqrt{x^2(s) + \dot{x}^2 }" />

## Kinematic feed forward


