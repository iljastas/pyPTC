from sympy import *


####################### Ackermann #######################
### Error Model with n=2, input=delta_f, disturbances=alpha_f, alpha_r
###############################################
var('y_e theta_e')      # States
var('alpha_f alpha_r')      # Disturbances (side slip angles)
var('v lw delta_f lo')  # Variales

# Define States
x1 = y_e
x2 = theta_e
x = Matrix([x1, x2])

# Nonlinear state-space representation with input: u = tan(delta_f + alpha_f)
a = Matrix([[v * sin(x2 + alpha_r) / cos(alpha_r) - lo * cos(x2 + alpha_r) * v / lw * tan(alpha_r)],
            [-v/lw * tan(alpha_r)]])
b = Matrix([lo * cos(x2 + alpha_r) * v/lw,  
            v / lw]) 

# Calculate the feedback linearization
lambda_ = Matrix([x1 - lo*sin(x2+alpha_r)]) # Flat output
z1 = lambda_                        # transformed state (1)
z2 = z1.jacobian(x) * a             # transformed state (2)
z = Matrix([[z1], [z2]])            # transformed state vector
alpha = z2.jacobian(x) * a          # state linearizing vectors (here: scalar)
beta = z2.jacobian(x) * b           # state linearizing vectors (here: scalar)



# Print solution
print("##################################")
print("Ackermann error Model with n=2, input=delta_f, disturbances=alpha_f, alpha_r")
print("z (state vector):")
pprint(z)
print("\nalpha:")
pprint(alpha)
print("\nbeta:")
pprint(beta)
print("##################################")
print("----------------------------------")
