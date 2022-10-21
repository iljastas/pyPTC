


'''
    DESCRIPTION: Class for robot parameter
    Author: Ilja Stasewisch, Date: 2019-03-26
'''

import numpy as np

class Robot_Parameters():
    def __init__(self, wheelbase=3.0,
                       length_offset=-1.0,
                       min_velocity=0.1, max_velocity=5.0, Tt_steer=0.02, T_PT1_yaw = 0.2, V_PT1_yaw=0.95):
        self._sampling_time = 0.01
        self._T_1order_steer = 0.2
        self._type = "not defined"

    def sampling_time(self):
        return self._sampling_time
    def length_wheelbase(self):
        return self._wheelbase
    def T_1stOrderLag_steer(self):
        return self._T_1order_steer
    def T_2ndOrderLag_steer(self):
        return self._T1_2order_steer, self._T2_2order_steer
    def T_1stOrderLag_velocity(self):
        return self.T_v


    def max_steerAngle(self):
        return self._max_steerAngle
    def max_velocity(self):
        return self._max_velocity

    def length_offset(self):
        return self._length_offset

    def type(self):
        return self._type

class Ackermann_Parameters(Robot_Parameters) :
    def __init__(self,
                wheelbase=3.0, length_offset=-1.0, look_ahead=0.0, # Length
                max_steerAngle=np.deg2rad(35.0), T_steer=0.375, des_velocity=0.1, min_velocity=0.1, max_velocity=5.0,
                T2_PT2_steer = 0.02, T1_PT2_steer = 0.19, steer_gradient=np.deg2rad(70.0), Tt_steer=0.02,
                T_PT1_yaw = 0.1, V_PT1_yaw=.95,
                T_PT1_vel = 0.15) :  # Velocity
        
        super().__init__()
        
        self._type = "Ackermann"

        # Geometry
        self._wheelbase = wheelbase
        self._length_offset = length_offset
        self.look_ahead = look_ahead
                
        # Velocity
        self._max_velocity = max_velocity
        self.desired_velocity = des_velocity
        self.mean_velocity = (min_velocity + max_velocity) / 2.0

        # Steering
        self._max_steerAngle = max_steerAngle
        self._T_1order_steer = T_steer
        self.T2_PT2_steer = T2_PT2_steer
        self.T1_PT2_steer = T1_PT2_steer
        self.steer_gradient = steer_gradient # rad/s
        self.tDead_steer = Tt_steer
        self.ramp_delta = steer_gradient # rad/s

        # Yaw Dynamic
        self.T_PT1_yaw = T_PT1_yaw
        self.V_PT1_yaw = V_PT1_yaw

        # Velocity/Longitundinal Dynamic
        self.T_PT1_vel = T_PT1_vel


