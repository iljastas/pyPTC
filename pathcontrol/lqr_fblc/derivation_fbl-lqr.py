from sympy import *


#######################

var('y_e theta_e')      # States
var('v lw delta_f lo')  # Variales

# Define States
x = Matrix([y_e, theta_e])
x1 = y_e
x2 = theta_e

# Nonlinear state-space representation with input: u = tan(delta_f)
a = Matrix([v * sin(x2),
            0])
b = Matrix([lo * v / lw  * cos (x2),
            v / lw]) 

# Calculate the feedback linearization
lambda_ = Matrix([x1 - lo*sin(x2)]) # Flat output
z1 = lambda_                        # transformed state (1)
z2 = z1.jacobian(x) * a             # transformed state (2)
alpha = z2.jacobian(x) * a          # state linearizing vectors (here: scalar)
beta = z2.jacobian(x) * b           # state linearizing vectors (here: scalar)

# Print solution
print("\z1:")
pprint(alpha)
print("\nz2:")
pprint(alpha)
print("\nalpha:")
pprint(alpha)
print("\nbeta:")
pprint(beta)
