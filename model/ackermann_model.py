#!/usr/bin/env python3

# PYTHON
import numpy as np
from numpy import sin, cos, tan
from .robot_model import Robot_Model


class Ackermann(Robot_Model) :

    def __init__(self, robot_state, robot_parameters,
                       steer_model="1st-order-lag", yaw_model=None, velocity_model=None) :
        super().__init__()

        self._parameters = robot_parameters
        self._robot_state = robot_state

        self._yaw_model = yaw_model
        self._steer_model = steer_model
        self._velocity_model = velocity_model

    def step(self, steer_input, vel_input, Ts=None, sideslip_angles=[0.0, 0.0]) :
        if type(steer_input) != list:
             steer_input = [steer_input, 0.0] # front and rear steering angle
        Ts = self._parameters.sampling_time()
        state = self._robot_state.as_dict()

        # Position
        av = sideslip_angles[0]
        ah = sideslip_angles[1]
        xNew = state["x"] + Ts * state["v"] * cos(state["yaw"] + ah) / cos(ah)
        yNew = state["y"] + Ts * state["v"] * sin(state["yaw"] + ah) / cos(ah)
        # Front Steering
        steerNew, steerRateNew = self.calc_steerModel(state, steer_input)
        steerNew = self.saturation(steerNew, self._paramters.max_steerAngle())
        # Rear Steering (if vehicle is allwheel-steered)
        steerNew, steerRateNew = self.calc_steerModel(state, steer_input)
        steerNew = self.saturation(steerNew, self._paramters.max_steerAngle())
        # Orientation
        yawNew, yawRateNew = self.calc_yawModel(state, sideslip_angles)
        # Velocity
        vNew, aNew = self.calc_velocityModel(state, vel_input)
        vNew = self.saturation(vNew, self._paramters.max_velocity())

        self.update_states(x=xNew, y=yNew, yaw=yawNew, v=vNew, # minimum model states
                           steer=steerNew, yawRate=yawRateNew) # additional states

    def calc_velocityModel(self, state, vel_desired):
        if self._velocity_model is None:
            vNew = vel_desired
            accelNew = 0.0
        elif self._velocity_model == "1st-order-lag":
            T_v = self._paramters.T_1stOrderLag_velocity()
            Ts = self._paramters.sampling_time()
            accelNew = 1.0 / T_v * (vel_desired - state["v"])
            yawNew = state["v"] + Ts * accelNew 
        
        return vNew, accelNew

    def calc_yawModel(self, state, alpha):
        Ts = self._paramters.sampling_time()
        lw = self._paramters.length_wheelbase()

        if self._yaw_model is None:
            yawRateNew = state["v"]/lw * (tan(state["steer"] + alpha[0]) - tan(alpha[1]) )
            yawNew = state["yaw"] + Ts * yawRateNew
        
        elif self._yaw_model == "1st-order-lag":
            # yaw
            yawNew = state["yaw"] + Ts * state["yawRate"] # just a integration
            # yaw-rate
            T_yaw = self._paramters.timeConstant_yaw_1stOrderLag()
            yaw_desired = state["v"]/lw * (tan(state["steer"] + alpha[0]) - tan(alpha[1]) )
            yawRateNew = state["yawRate"] + Ts / T_yaw * (yaw_desired - state["yawRate"])

        return yawNew, yawRateNew

    def calc_steerModel(self, state_actual, steer_input, steerRate_actual):
        Ts = self._paramters.sampling_time()

        if self._steer_model is None:
            steerNew = steer_input
            steerRate = 0.0

        elif self._steer_model == "1st-order-lag":
            T_steer = self._paramters.T_1stOrderLag_steer()
            steerRateNew = 1 / T_steer * (steer_input - state_actual)
            steerNew = state_actual + Ts * steerRateNew

        elif self._steer_model == "2st-order-lag":
            T1_steer, T2_steer = self._paramters.timeConstants_steer_2ndOrderLag()
            steerRateNew = steerRate_actual + 1 / T2_steer * (steer_input - T1_steer * steerRate_actual - state_actual)
            steerNew = state_actual + Ts * steerRate_actual # just a integration

        return steerNew, steerRateNew

    def saturation(self, value, max_value): # Limit steering angle or velocity
        if np.fabs(value) > max_value: 
            return max_value * np.sign(value)
        else :
            return value

    def update_states(self, x, y, yaw, v, steer, steerRate=0.0, yawRate=0.0):
        # print(v)
        self._robot_state.update(x=x, y=y, yaw=yaw, velocity=v,
                                 steerAngle=steer)

# main for debug
if __name__ == '__main__': 
    oAckermann = Ackermann()
    oAckermann.step(0.1, 0.5)