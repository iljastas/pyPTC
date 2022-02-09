# Feed forward for Ackerman-steered (car-like) vehicles

The path (its x- and y-positions) is time indepedent, but a path tracking control and its feed forward are time depedent.

Here, flatness-based feedforward controls are derived, because for nonlinear systems it is more accurate.

The flat ouput is <img src="https://latex.codecogs.com/svg.image?y_f&space;=&space;\begin{bmatrix}&space;a&space;&&space;b&space;&space;\\\end{bmatrix}" title="y_f = \begin{bmatrix} a & b \\\end{bmatrix}" />

## Kinematic feed forward


