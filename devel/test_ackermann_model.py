
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

from pathcontrol.control_base import Control_Base
from pathcontrol.lqr_fblc.lqr_fblc import LQR_FBL


# Define Vehicle-Model
params = Ackermann_Parameters()
state = Robot_State(params)
model = Ackermann(state, params)

# Define Path and Vehicle-Model
path = Path(state, params, path_name="PythonRobotics.lqr_steer_control") # xy-Path with handler


# Define Control
# feedforward = Feedforward(path)
feedforward = None
control = Control_Base()
control = LQR_FBL(params, Ts=0.01)


# Simulate Path tracking control
sim = Simulation(model, control, path, Ts=0.01)
# sim = Simulation(model, control, path)
sim.run()

