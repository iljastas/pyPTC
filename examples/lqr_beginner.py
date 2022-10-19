import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")


# LOCAL
from model.ackermann_model import Ackermann
from model.robot_state import Robot_State
from model.robot_parameters import Ackermann_Parameters

from pathplanning.path import Path()

# Model
params = Ackermann_Parameters()
state = Robot_State(params)
model = Ackermann(state, params)

# Control
path = Path()
control = LQR()
feedforward = Feedforward()

# Execute
sim = Simulation(model, path, control, feedforward)
sim.run()



