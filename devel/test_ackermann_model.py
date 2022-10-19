
#
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")

# GLOBAL
import numpy as np
from itertools import count

# LOCAL
from model.ackermann_model import Ackermann
from model.robot_state import Robot_State
from model.robot_parameters import Ackermann_Parameters
from model.simulation import Simulation

from pathplanning.path import Path


# Define Path and Vehicle-Model
path = Path(path_name="PythonRobotics.lqr_steer_control") # xy-Path with handler


# Define Vehicle-Model
params = Ackermann_Parameters()
state = Robot_State(params)
model = Ackermann(state, params)

# Define Control
# feedforward = Feedforward(path)
# control = LQR(feedforward)

control = None

# Simulate Path tracking control
sim = Simulation(model, control, path)
# sim = Simulation(model, control, path)
sim.run()

