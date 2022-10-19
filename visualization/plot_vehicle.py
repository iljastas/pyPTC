
import matplotlib.pyplot as plt
import numpy as np
from numpy import sin, cos



def rotmat_2D(angle):
    return np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
def translate_2D(x, y):
    return np.array([x, y]).reshape(2,1)
def transform_wheel(wheel_shape, rot_yaw, rot_steer, local_xy, global_xy):
    wheel_tf = rot_steer @ wheel_shape
    wheel_tf = local_xy + wheel_tf # first translate
    wheel_tf = rot_yaw @ wheel_tf
    wheel_tf = np.add(wheel_tf, global_xy)
    return wheel_tf

def plot_acker(ax, x, y, yaw, steer_front, steer_rear=0.0, params=None):
    if params is not None:
        wheelbase = params.length_wheelbase()
        trackwidth = wheelbase / 2.0
        tirewidth = wheelbase / 10.0
        tire_length = wheelbase / 5.0
    else:
        wheelbase = 3.0
        trackwidth = 1.5
        tirewidth = 0.3
        tire_length = 0.6

    rotmat_yaw = rotmat_2D(yaw)
    rotmat_steerFront = rotmat_2D(steer_front)
    rotmat_steerRear = rotmat_2D(steer_rear)

    boards = np.array([[0.0, trackwidth],
                        [wheelbase*2, trackwidth],
                        [wheelbase*2, -trackwidth],
                        [0.0, -trackwidth],
                        [0.0, trackwidth]]) / 2.0

    
    boards = rotmat_yaw @ boards.T
    boards = np.add(boards, translate_2D(x, y))

    wheel = np.array([[-tire_length, tirewidth],
                    [tire_length, tirewidth],
                        [tire_length, -tirewidth],
                        [-tire_length, -tirewidth],
                        [-tire_length, tirewidth]]).T / 2.0
    
    # trans_wheel = 

    

    wheel_frontLeft = transform_wheel(wheel, rotmat_yaw, rotmat_steerFront,
                                      translate_2D(wheelbase, -trackwidth/2.0), translate_2D(x,y))
    wheel_frontRight = transform_wheel(wheel, rotmat_yaw, rotmat_steerFront,
                                       translate_2D(wheelbase, trackwidth/2.0), translate_2D(x,y))
    wheel_rearLeft = transform_wheel(wheel, rotmat_yaw, rotmat_steerRear,
                                     translate_2D(0.0, trackwidth/2.0), translate_2D(x,y))
    wheel_rearRight = transform_wheel(wheel, rotmat_yaw, rotmat_steerRear,
                                      translate_2D(0.0, -trackwidth/2.0), translate_2D(x,y))
    

    ax.plot(boards[0,:], boards[1,:], color='k')
    ax.plot(wheel_frontLeft[0,:], wheel_frontLeft[1,:], color='k')
    ax.plot(wheel_frontRight[0,:], wheel_frontRight[1,:], color='k')
    ax.plot(wheel_rearRight[0,:], wheel_rearRight[1,:], color='k')
    ax.plot(wheel_rearLeft[0,:], wheel_rearLeft[1,:], color='k')
    ax.axis("equal")
    
    
def plot_skid(x, y, yaw, velocity):
    pass
def plot_arti():
    pass
