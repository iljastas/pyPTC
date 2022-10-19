# -*- coding: utf-8 -*-
import numpy as np
from numpy.linalg import inv
import math
from math import sin, cos
import sys
from threading import Thread, Lock
import matplotlib.pyplot as plt
from collections import namedtuple
import copy

import sys
import os
main_path = os.path.dirname("/home/ilja/02_diss_ws/src/04_path_tracking/pathtracking_all/src/")
sys.path.append(main_path) 

# LOCAL




'''##########################################################################
##############    PATH class for handling control-path    ###################
##########################################################################'''



class Path:  

    def __init__(self, path_name="line", path_numpy_xy_2xN=None):
        pass

        if path_name == "PythonRobotics.lqr_steer_control":
            from PythonRobotics.PathPlanning.CubicSpline.cubic_spline_planner import calc_spline_course

            ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
            ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
            x, y, yaw, curve, s = calc_spline_course(ax, ay, ds=0.1)

            self.path = np.vstack( (x,y,yaw) )
            print(self.path.shape)


        self._x = x
        self._y = y

    def x(self):
        return self._x
    def y(self):
        return self._y

    def init_diss(self, x, y, robot_state, robot_param, vel = 1.0, length_path_end=5.0) :
        # Class Variables
        self._mutex = Lock()

        # Arguments
        self.x = x
        self.y = y
        self.wheelbase = robot_param.wheelbase
        # self.offset = robot_param.length_offset
        self.offset = -1.5
        self.min_length_goal_reached = length_path_end
        self.robot_state = robot_state

        # Calculate related path element/stuff/foo
        # print("asd")
        self.dx, self.dy    = self._calc_dot()
        self.ddx, self.ddy  = self._calc_ddot()
        self.yaw            = self._calc_yaw(self.dx, self.dy, vel)
        # print(np.mean(self.yaw))

        self.curve          = self._calc_curve(self.dx, self.ddx, self.dy, self.ddy)
        self.steer          = self._calc_steer(self.curve, vel)
        self.x_offset, self.y_offset            = self._calc_offset_path(self.x, self.y, self.yaw)
        self.arclength, self.ds, self.ds_mean   = self._calc_arclength(self.dx, self.dy)

        # Set Goal
        self.goal = xyYaw_to_matrix(self.x_offset[-1], self.y_offset[-1], self.yaw[-1])
        self.goalInv = inv(self.goal)
        
        self.meters_around_last_nnidx = 0.5 * np.fabs(vel) # Multiplication with the desired/mean velocity for normalization

        self.reset_me()

        # plt.plot(self.x, self.y)
        # plt.plot(self.x_offset, self.y_offset)
        # plt.legend(["planned", "offset"])
        # plt.show()


    def update_lengthoffset(self, length_offset):
        self.offset = length_offset
        
    def reset__index_nn(self):
        self.index_nn = -1
        self.index_nn_lh = -1

    def reset_me(self):
        self.new_round = True
        self.lateral_error = np.full(shape=(len(self.arclength),), fill_value=np.NAN) 
        self.yaw_error = np.full(shape=(len(self.arclength),), fill_value=np.NAN) 
        self.index_nn = -1
        self.index_nn_lh = -1
        self.drivenPath = DrivenPath( )
        self.referencePath = None
        self.predictionPath__dlqt_mpc = None
        self.referencePath__dlqt_mpc = None

    ######### EXTERN GET FUNCTION ########
    def get_nnPosition(self):
        with self._mutex:
            if self.index_nn < 0 :
                return [self.x_offset[0], self.y_offset[0]]
            else :
                return [self.x_offset[self.index_nn], self.y_offset[self.index_nn]]

    # def get_referencePath(self, path_length, delta_length, prediction_steps):
    def get_referencePath(self, *argv):
        if len(argv) == 0:
            return self.referencePath
        elif len(argv) == 2:
            path_length = argv[0]
            prediction_steps = argv[1]
            delta_length = path_length / (prediction_steps-1)
            refPath = self._determine_referencePath(path_length, delta_length, prediction_steps)
            with self._mutex:
                self.referencePath = refPath
            return refPath            
        elif len(argv) == 3:
            path_length = argv[0]
            delta_length = argv[1]
            prediction_steps = argv[2]
            refPath = self._determine_referencePath(path_length, delta_length, prediction_steps)
            with self._mutex:
                self.referencePath = refPath
            return refPath
        else:
            print("Error in Path_Handler.get_referencePath(): Too less or many arguments!")

    def get_predictionPath__dlqt_mpc(self):
        with self._mutex:
            return self.predictionPath__dlqt_mpc
    def get_referencePath__dlqt_mpc(self):
        with self._mutex:
            return self.referencePath__dlqt_mpc

    def get_xyYaw_byIdx(self, index):
        state = np.array([self.x_offset[index], self.y_offset[index], self.yaw[index]]).reshape(-1,1)
        return state

    def get_lookaheadPose(self):
        with self._mutex:
            if self.index_nn_lh < 0 :
                return [self.x_offset[0], self.y_offset[0]]
            else :
                return [self.x_offset[self.index_nn_lh], self.y_offset[self.index_nn_lh], self.yaw[self.index_nn_lh] ]

    def get_lookaheadSteer(self):
        with self._mutex:
            if self.index_nn_lh < 0 :
                return [self.steer[0], self.steer[0]]
            else :
                return self.steer[self.index_nn_lh]

    def get_lookaheadPosition(self):
        with self._mutex:
            if self.index_nn_lh < 0 :
                return [self.x_offset[0], self.y_offset[0]]
            else :
                return [self.x_offset[self.index_nn_lh], self.y_offset[self.index_nn_lh]]

    def get_feedforwardSteerRate(self):
        with self._mutex:
            return self.steerRate[self.index_nn]
    def get_feedforwardSteer(self):
        with self._mutex:
            return self.steer[self.index_nn]
    def get_offsetPath(self, index=None) :
        with self._mutex:
            if index is None:
                return self.x_offset, self.y_offset, self.yaw
            else:
                return self.x_offset[index], self.y_offset[index], self.yaw[index]
    def get_path(self) :
        with self._mutex:
            return self.x, self.y, self.yaw
    def get_arclength(self) :
        with self._mutex:
            return self.arclength
    def get_lateralError(self) :
        with self._mutex:
            if self.new_round==True:
                # print("how many non nans:", np.count_nonzero(~np.isnan(self.lateral_error)))
                self.new_round = False
            return self.lateral_error
    def get_yawError(self) :
        with self._mutex:
            return self.yaw_error
    def get_drivenPath(self) :
        with self._mutex:
            return self.drivenPath.xo, self.drivenPath.yo, self.drivenPath.yaw         
    def get_drivenArcLength(self) :
        with self._mutex:
            return self.drivenPath.arclength        

    def plot_me(self, cmd="now"):
        with self._mutex:
            fig, ax = plt.subplots(nrows=2, ncols=2)
            fig.canvas.set_window_title("Path Handler - Path Information")
            
            ax[0,0].plot(self.x_offset, self.y_offset, color='b')
            ax[0,0].plot(self.x, self.y, color='k')

            
            ax[0,1].plot( self.arclength[0:-1], self.curve )
            
            if cmd == "now":
                plt.show()


    ####################################################################
    ######################### EXECUTE FUNCTION #########################
    def execute(self, look_ahead=None, purepursuit=False, wheelloader=False) :
        with self._mutex:
            if wheelloader:
                l10nOffset = self.robot_state.get_l10nOffset()
            else:
                l10nOffset = self.robot_state.get_l10nOffset()

            # Nearest Index for Visualization
            index_nn, dL, nearestPath_matrix, nearest_xyYaw = self._get_indexNearestPosition(l10nOffset)
            self.nearest_xyYaw = nearest_xyYaw
            # print("index_nn:", index_nn)
            self._update_index_nn(index_nn)
            # Error and Driven Path
            diff_error_matrix_nn = self._get_errorMatrix(nearestPath_matrix, l10nOffset)
            # print("diff_error_matrix_nn[1,3]:", diff_error_matrix_nn[1,3])
            self._update_lateralError(index_nn, diff_error_matrix_nn[1,3])
            self._update_yawError(index_nn, matrix_to_yaw(diff_error_matrix_nn) )
            self._update_drivenPath(l10nOffset)

            # Nearest Index for  Control
            if look_ahead is None:
                index_nn_lh = index_nn
                diff_error_matrix_lh = diff_error_matrix_nn
                self._update_indexNnLookahead(index_nn_lh)
            if purepursuit == False:
                index_nn_lh, _, des_lah_pose_matrix, lah_xyYaw = self._get_indexNearestPosition(l10nOffset, look_ahead)
                diff_error_matrix_lh = self._get_errorMatrix(des_lah_pose_matrix, l10nOffset)
                self._update_indexNnLookahead(index_nn_lh)
            else:
                index_nn_lh, _, des_lah_pose_matrix = self._get_indexNearestPosition_byCircle(l10nOffset, look_ahead)
                diff_error_matrix_lh = self._get_errorMatrix(des_lah_pose_matrix, l10nOffset, in_robotframe=True)
                self._update_indexNnLookahead(index_nn_lh)
                

            # Behind Goal
            velocity = self.robot_state.get_velocity()
            behind_goal = self._is_behindGoal(l10nOffset, velocity)
            
        return diff_error_matrix_lh, behind_goal, index_nn, index_nn_lh, dL, diff_error_matrix_nn
    ####################################################################
    ####################################################################


    def _is_behindGoal(self, robot_state, velocity):

        if self.arclength[-1]*0.95 > self.drivenPath.arclength[-1]:
            return False
        # print(self.drivenPath.arclength[-1])

        xo, yo, yaw = robot_state[0], robot_state[1], robot_state[2]
        actual = xyYaw_to_matrix(xo, yo, yaw)
        goal2actual = self.goalInv @ actual
        length_to_goal = np.hypot( goal2actual[0,3], goal2actual[1,3] )

        self.length_to_goal = length_to_goal

        if length_to_goal > self.min_length_goal_reached :
            return False
        else :
            
            dx = goal2actual[0, 3]

            if velocity > 0.0 and dx > 0.0:
                # print("asdassd")
                return True
            elif velocity < 0.0 and dx < 0.0:
                return True 
            else :
                return False               


    def _get_errorMatrix(self, desired, l10nOffset, in_robotframe=False) :
        xo, yo, yaw = l10nOffset[0], l10nOffset[1], l10nOffset[2]
        actual = xyYaw_to_matrix(xo, yo, yaw)

        if in_robotframe == False:
            diff_tf = inv(desired) @ actual
        else: # in Robot-Frame
            diff_tf = inv(actual) @ desired
            pass

        return diff_tf

    def _get_indexNearestPosition_byCircle(self, l10nOffset, lookahead_distance):

        idx_around_last_nn = int( self.meters_around_last_nnidx / np.fabs(self.ds_mean)+1  )
        # Define Start-Idx for the Loop

        if  self.index_nn_lh  < 0:
            current_xy = np.vstack( (l10nOffset[0], l10nOffset[1]) ).reshape(1,-1)
            self.index_nn_lh = self._init_index_nn(current_xy)
            idx_around_last_nn = int( 10.0 / np.fabs(self.ds_mean)+1  )


        start_index = self.index_nn_lh - idx_around_last_nn
        if start_index < 0 :
            start_index = 0
        # Define End-Idx for the Loop
        if self.index_nn_lh < 0: # self.index_lookahead is not inited
            end_index = len(self.x)
        else: 
            end_index = self.index_nn_lh + idx_around_last_nn
        if end_index > len(self.x) :
            end_index = len(self.x)

        # GET DESIRED-POSE
        dist_to_lookahead = float("inf")
        path_desired_idx = self.index_nn_lh
        actual_tfMatrix = xyYaw_to_matrix(l10nOffset[0], l10nOffset[1], l10nOffset[2])
        for i in range(start_index, end_index) :           
            iPath_tfMat = xyYaw_to_matrix(self.x_offset[i], self.y_offset[i], self.yaw[i])
            dist_offset2path = getLength2DSign(actual_tfMatrix, iPath_tfMat)
            if dist_offset2path < 0.0 :
                continue
            else :
                eDist = dist_offset2path - lookahead_distance
                
                if eDist <= 0.0 :
                    continue
                elif eDist < dist_to_lookahead :
                    dist_to_lookahead = eDist
                    path_desired_idx = i
                    if np.fabs(dist_to_lookahead) <  np.fabs(self.ds_mean):
                        break

       
        desired_pose_matrix = xyYaw_to_matrix(self.x_offset[path_desired_idx], self.y_offset[path_desired_idx], self.yaw[path_desired_idx])
        return path_desired_idx, dist_offset2path, desired_pose_matrix


    def _get_indexNearestPosition(self, robot_state, look_ahead=None):
        # print(robot_state[0], robot_state[1], robot_state[2])
        robot_state_lh = [robot_state[0], robot_state[1], robot_state[2] ]
        # print("robot_state_lh:", robot_state_lh, type(robot_state_lh), look_ahead)
        # sleep(11.1)
        if look_ahead is not None:
            # print("use look ahead")
            # print("before lh:", robot_state[0] )
            # print("\t debug:", robot_state_lh[0] + look_ahead * np.cos(robot_state[2]))
            robot_state_lh[0] = robot_state_lh[0] + look_ahead * cos(robot_state[2])
            robot_state_lh[1] = robot_state_lh[1] + look_ahead * sin(robot_state[2])
            # print("after lh:", robot_state[0] )

            last_nn_idx = self.index_nn_lh

        else :
            # print()
            last_nn_idx = self.index_nn

        if last_nn_idx < 0:
            # print("\trobot_state_lh:", robot_state_lh, last_nn_idx, type(robot_state_lh), robot_state_lh[0])
            current_xy = np.array(robot_state_lh)
            current_xy = current_xy[0:-1].reshape(1,-1)
            current_xy = np.array( [robot_state_lh[0], robot_state_lh[1]] ).reshape(1,-1)
            # print("current_xy", current_xy, type(current_xy), current_xy.shape)
            # try:
            last_nn_idx = self._init_index_nn(current_xy)
            # except:
                # last_nn_idx = 0

        # Define Start-Index and End-Index for the Loop       
        start_index = last_nn_idx - int(self.meters_around_last_nnidx /  np.fabs(self.ds_mean)+1 )
        if start_index < 0 : start_index = 0

        if last_nn_idx < 0: # at init, search in whole path
            end_index = len(self.x)-1
        else: 
            end_index = last_nn_idx + int( self.meters_around_last_nnidx / np.fabs(self.ds_mean)+1  )
            if end_index > len(self.x) : end_index = len(self.x)

        # Get index for neatest euclidean distance
        dL = float('Inf')
        index = 0
        xo, yo, yaw = robot_state_lh[0], robot_state_lh[1], robot_state_lh[2]
        # print("start_index: ", start_index)
        # print("end_index: ", end_index)
        # print("self.meters_around_last_nnidx: ", self.meters_around_last_nnidx)
        # print("int( self.meters_around_last_nnidx / np.fabs(self.ds_mean) = ", int( self.meters_around_last_nnidx / np.fabs(self.ds_mean) +1) )
        for i in range(start_index, end_index) :
            dL_ = np.hypot(self.x_offset[i] - xo, self.y_offset[i] - yo)
            if dL_ < dL :
                index = i
                dL = dL_

        # xyYaw = [self.x[index], self.y[index], self.yaw[index] ]
        pose_matrix = xyYaw_to_matrix(self.x_offset[index], self.y_offset[index], self.yaw[index])

        des_xyYaw = [self.x_offset[index], self.y_offset[index], self.yaw[index]]

        return index, dL, pose_matrix, des_xyYaw


    def _init_index_nn(self, current_xy):
        # print(self.x_offset.shape)
        # t = self.x_offset.reshape(1, -1)
        xo = self.x_offset
        yo = self.y_offset
        # print(xo.shape)
        # print(yo.shape)
        tree = spatial.KDTree( list(zip(xo, yo)) )
        lat_error, idx = tree.query(current_xy, k=1)
        # ic("init by kdtree:", lat_error, idx, current_xy)
        # ic(self.x_offset[0], self.y_offset[0])
        # ic(idx, type(idx))
        if np.isscalar(idx):
            return idx
        else:
            return idx[0]
        

    def _determine_referencePath(self, path_length, delta_length, prediction_steps):
        '''
            FOR MPC or DQN, Reference path 
        '''
        with self._mutex:
            idx = copy.deepcopy(self.index_nn)
            # print("start idx:", idx)
            x, y, yaw, steer, steerRate = np.empty(shape=(5, prediction_steps))
            x[0]         = self.x_offset[idx] 
            y[0]         = self.y_offset[idx] 
            yaw[0]       = self.yaw[idx] 
            steer[0]     = self.steer[idx] 
            steerRate[0] = self.steerRate[idx] 
            iter, iLength, idL = 1, 0.0, 0.0
            idx += 1
            while iLength <= path_length and iter < prediction_steps:
                # Path Determination
                if idx >= len(self.ds):
                    # print("WAS MACHT DAS?")
                    idx = len(self.ds) - 1

                idL += self.ds[idx]
                if idL >= delta_length:
                    x[iter]         = self.x_offset[idx] 
                    y[iter]         = self.y_offset[idx] 
                    yaw[iter]       = self.yaw[idx] 
                    steer[iter]     = self.steer[idx] 
                    steerRate[iter] = self.steerRate[idx] 
                    idL = 0.0
                    iter += 1
                    # ic(iLength)

                # Abort Criteria
                iLength += self.ds[idx]
                idx += 1
            
            if idL > 0.0 and idx < self.x_offset.size: # if last pose was not set, because of "bad" dL in apth
                x[iter]         = self.x_offset[idx] 
                y[iter]         = self.y_offset[idx] 
                yaw[iter]       = self.yaw[idx] 
                steer[iter]     = self.steer[idx] 
                steerRate[iter] = self.steerRate[idx]
            elif idL > 0.0 and idx >= self.x_offset.size:
                idx = -1
                x[iter]         = self.x_offset[idx] 
                y[iter]         = self.y_offset[idx] 
                yaw[iter]       = self.yaw[idx] 
                steer[iter]     = self.steer[idx] 
                steerRate[iter] = self.steerRate[idx]

            # ic(iLength, iter, path_length, prediction_steps, delta_length)
            # ic("####################################", x[-1])
            # ic(iLength, delta_length)
            ret = {"x": x, "y": y, "yaw": yaw, "steer": steer, "steerRate": steerRate}
            return ret


    ######################################################################################
    ########################## Set Functions #############################################
    def set_predictionPath__dlqt_mpc(self, path):
        with self._mutex:
            self.predictionPath__dlqt_mpc = path
    def set_referencePath__dlqt_mpc(self, path):
        with self._mutex:
            self.referencePath__dlqt_mpc = path

    ######################################################################################
    ########################## UPDATE CLASS VARIABLES ####################################
    def _update_index_nn(self, index_nearest_neighbour):
        self.index_nn = index_nearest_neighbour
    def _update_indexNnLookahead(self, index_nn_lookahead):
        self.index_nn_lh = index_nn_lookahead
    def _update_lateralError(self, index, y_error):
        if self.lateral_error[index] is None    or    y_error < self.lateral_error[index]    or     np.isnan(self.lateral_error[index]) == True:
            self.lateral_error[index] = y_error
    def _update_yawError(self, index, yaw_error):
        if self.yaw_error[index] is None  or yaw_error < self.yaw_error[index] or np.isnan(self.yaw_error[index]) == True:
            self.yaw_error[index] = yaw_error
    def _update_drivenPath(self,robot_state):
        self.drivenPath.set_robotState(robot_state)

    ######################################################################################
    ########################## CALCULACTE PATH FUNCTIONS #################################
    def _calc_arclength(self, dx, dy) :
        ds = [ np.sqrt(idx**2.0 + idy**2.0) for (idx, idy) in zip(dx, dy)]
        ds_mean = np.mean(ds)
        arclength = np.array([0.0])
        arclength = np.hstack( (arclength, np.cumsum(ds)) )
        # print("arclength = ", arclength.shape)
        return arclength, ds, ds_mean

    def _calc_offset_path(self, x, y, yaw) :
        # print(self.offset, np.rad2deg(yaw[0]))
        x_offset = x + self.offset * np.cos(yaw)
        y_offset = y + self.offset * np.sin(yaw)
        return x_offset, y_offset

    def _calc_dot(self) :
        dx = np.diff(self.x)
        dy = np.diff(self.y)
        dx = np.append(dx, [dx[-1]], axis=0)
        dy = np.append(dy, [dy[-1]], axis=0)
        return dx, dy

    def _calc_ddot(self) :
        ddx = np.diff(self.dx)
        ddy = np.diff(self.dy)
        ddx = np.append(ddx, [ddx[-1]], axis=0)
        ddy = np.append(ddy, [ddy[-1]], axis=0)
        return ddx, ddy

    def _calc_yaw(self, dx, dy, velocity) :
        yaw = np.arctan2(self.dy, self.dx)
        if velocity < 0.0:
            # pass
            # print("_calc_yaw: if velocity < 0.0:")
            yaw = [minusPi_to_pi(iyaw- math.pi) for (iyaw) in yaw]
            # yaw = np.asarray(self.yaw)
        # for iyaw in yaw:
            # if np.fabs(iyaw) > 0.2:
                # print("iyaw>0.2", iyaw)
        return yaw

    def _calc_steer(self, curve, velocity) :
        steer = np.arctan2(curve * self.wheelbase, 1)
        if velocity < 0.0:
            steer = steer * -1.0
            steer = np.asarray(steer)

        return steer

    def _calc_curve(self,dx, ddx, dy, ddy) :
        curve = (dx * ddy - ddx * dy ) / ((dx**2.0 + dy**2.0) ** (3.0 / 2.0))
        return curve

    def _calc_steerRate(self, steer, ds, v) :
        dsteer = np.diff(steer)
        dsteer = np.append(dsteer, [dsteer[-1]], axis=0)
        steerRate = dsteer / ds * v
        steerRate = np.asarray(steerRate)
        return steerRate
