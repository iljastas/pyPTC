'''
    Path-Tracking Control with Feedback Linearization (FBL) and LQR
    Author: Ilja Stasewitsch, Date: 2022-10-19
'''
import numpy as np
from numpy.linalg import inv
from threading import Thread, Lock
from numpy import fabs, sin, cos, tan

from PythonRobotics.PathTracking.lqr_steer_control import dlqr

class LQR_FBL:
    
    def __init__(self, robot_parameters, Ts, model_type="", q_yE=10, q_yawE=5, q_steer=1, q_steerRate=1) :
        print('Path Tracking Control: LQR with Feedback Linearization (FBL)')
        self._goal_reached = False
        self.robot_param = robot_parameters
        self._Ts = Ts

        self.ctrlMatrix_K = None
        self.ctrlMatrix_K_yE = None
        self.ctrlMatrix_K_thetaE = None
        self.model_type = model_type
        self.velocity = 1.0
        self.steer = 0.0
        self.q_yE = q_yE
        self.q_yawE = q_yawE
        self.q_steer = q_steer
        self.q_steerRate = q_steerRate 
        self.dVel = 0

        return

            # Calculate the Control
        # if self.ctrlMatrix_K is None:
        # try:
        #     Ts = self.sim_param.Ts_ctrl
        #     A = np.array( [[1, Ts, 0], [0, 1, Ts], [0, 0, 1]] ).reshape(3,3)
        #     B = np.array( [0, 0, Ts] ).reshape(3,1)
        #     Q = np.eye(3, 3)
        #     Q[0, 0] = self.q_yE
        #     Q[1, 1] = self.q_yawE
        #     Q[2, 2] = self.q_steer
        #     R = np.array( [self.q_steerRate] ).reshape(1,1)
        #     self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
        # except Exception as e:
        #     print(e)
            # print(Q)

    def calc_controlMatrix(self, A, B, Q, R) :
        K, _, _ = dlqr(A, B, Q, R)
        return K

    def  get_ctrlMtrx_K(self, n, m=1):
        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl

            if n == 2:           
                A = np.array( [[1, Ts], [0, 1]] )
                B = np.array( [0, Ts] ).reshape(2,1)
                Q = np.array( [[self.q_yE, 0], [0, self.q_yawE]] )
                R = np.array( [self.q_steer] ).reshape(1,1)
                self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
            # print(self.ctrlMatrix_K)
        return self.ctrlMatrix_K

    def execute(self, diff_pose, velocity, steer=0.0, T_steer=None, alpha_vh=None, steerRate=None) :
        # ACKERMANN
        if self.model_type == "Ackermann__u_frontSteering_rearDrive__n2" :
            return self._ackermann_frontSteering_RearDrive_n2(diff_pose, velocity)
        elif self.model_type == "Ackermann__u_frontSteering__rearDrive__n3" :
            return self._ackermann_frontSteering_RearDrive_n3(diff_pose, velocity, steer, T_steer)

            ## FENDT
        elif self.model_type == "_ackr_errmodel__pt2__u_steerrate" :
            return self._ackr_errmodel__pt2__u_steerrate(diff_pose, velocity, steer, steer_rate=steerRate)
        elif self.model_type=="_ackr_errmodel__pt1__u_steerrate_n4":
            return self._ackr_errmodel__pt1__u_steerrate_n4(diff_pose, velocity, steer)
        elif self.model_type=="_ackr_errmodel__pt2__u_steer__n4":
            return self._ackr_errmodel__pt2__u_steer__n4(diff_pose, velocity, steer, steer_rate=steerRate)




        elif self.model_type == "_ackermann_deltaV_vH_u2_n2" :
            return self._ackermann_deltaV_vH_u2_n2(diff_pose)
        elif self.model_type == "_ackermann_deltaV_vH__u_steer_accel__n2" :
            return self._ackermann_deltaV_vH__u_steer_accel__n2(diff_pose)

        elif self.model_type == "_acker__u_steerfront__drive_rear__alpha__n2" :
            return self._acker__u_steerfront__drive_rear__alpha__n2(diff_pose, velocity, alpha_vh)

        elif self.model_type == "_acker__u_steerRateFront__drive_rear__alpha__n3" :
            return self._acker__u_steerRateFront__drive_rear__alpha__n3(diff_pose, velocity, steer, alpha_vh)


        elif self.model_type == "_ackr_errmodel_frontsteer_velRear__u_steerRate__n3" :
            return self._ackr_errmodel_frontsteer_velRear__u_steerRate__n3(diff_pose, velocity, steer)

        # KNICK:
        elif self.model_type == "_knick__yawRear_vRear__n3" :
            return self._knick__yawRear_vRear__n3(diff_pose, velocity)
        elif self.model_type == "_knick_rearYaw_RearDrive_n2":
            return self._knick_rearYaw_RearDrive_n2(diff_pose, velocity)


        # SKID-STEERING
        elif self.model_type == "_skidsteer__u_deltaV__n2" :
            return self._skidsteer__u_deltaV__n2(diff_pose, velocity)
        elif self.model_type == "_skidsteer__u_deltaV__n3" :
            return self._skidsteer__u_deltaV__n3(diff_pose, velocity)
        else :
            print("FBLC: Control type does not exist!")
            return 0.0

    def _acker__u_steerRateFront__drive_rear__alpha__n3(self, diff_pose, velocity, steer, alpha_vh):
        vhx = velocity
        lo = self.robot_param.length_offset
        wb = self.robot_param.wheelbase
        Ts = self.sim_param.Ts_ctrl

        # DEFINE THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = steer
        ye = x1
        Ye = x2
        dv = x3
        av = alpha_vh[0]
        ah = alpha_vh[1]
        
        
        # STATE TRANSFORMATION
        z1 = ye - lo*sin(Ye + ah)
        z2 =  (vhx*sin(Ye + ah))/cos(ah)
        z3 = (vhx**2*cos(Ye + ah)*(tan(av + dv) - tan(ah)))/(wb*cos(ah))
        Z = np.array([z1, z2, z3]).reshape(-1,1)
        alpha = -(vhx**3*sin(Ye + ah)*(tan(av + dv) - tan(ah))**2)/(wb**2*cos(ah))
        alpha = np.array(alpha).reshape(1,1)
        beta = (vhx**2*cos(Ye + ah)*(tan(av + dv)**2 + 1))/(wb*cos(ah))
        beta = np.array( beta ).reshape(1,1)


        if self.ctrlMatrix_K is None:
            A = np.array( [[1, Ts, 0], [0, Ts, 1], [0, 0, 1] ] ).reshape(3,3)
            B = np.array( [0, 0, Ts] ).reshape(-1,1)
            Q = np.diag( [self.q_yE,  self.q_yawE, self.q_steer])
            R = np.array( [self.q_steerRate] ).reshape(1,1)
            ic(self.q_yE, self.q_yawE, self.q_steer, self.q_steerRate)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)

        ctrl_linear = -self.ctrlMatrix_K @ Z
        ctrl = (-alpha + ctrl_linear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        
        self.steer += Ts * ctrl[0,0]
        return self.steer


    def _acker__u_steerfront__drive_rear__alpha__n2(self, diff_pose, velocity, alpha_vh):
        vhx = velocity
        lo = self.robot_param.length_offset
        wb = self.robot_param.wheelbase

        # DEFINE THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        ye = x1
        Ye = x2
        av = alpha_vh[0]
        ah = alpha_vh[1]
        
        # STATE TRANSFORMATION
        z1 = ye - lo*sin(Ye + ah)
        z2 =  (vhx*sin(Ye + ah))/cos(ah)
        Z = np.array([z1, z2]).reshape(2,1)
        alpha = -(vhx**2*cos(Ye + ah)*tan(ah))/(wb*cos(ah))
        alpha = np.array(alpha).reshape(1,1)
        beta = (vhx**2*cos(Ye + ah))/(wb*cos(ah))
        beta = np.array( beta ).reshape(1,1)

        # print(alpha.shape, beta.shape)

        ctrl_linear = -self.get_ctrlMtrx_K(n=2, m=1) @ Z
        ctrl = (-alpha + ctrl_linear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        ctrl = np.arctan2(ctrl, 1) 

        return ctrl[0,0] - av


    def _knick__yawRear_vRear__n3(self, diff_pose, velocity):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        # lo = self.robot_param.length_offset
        lr = self.robot_param.wheelbase
        lv = self.robot_param.length_front
        steer = self.steer

        # GET THE REFERENCE-POSE
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)       
        x3 = steer
        y_e = diff_pose[1, 3]
        theta_e = matrix_to_yaw(diff_pose)       
        delta = steer


        # STATE TRANSFORMATION
        z1 = y_e
        z2 = v*sin(theta_e)
        z3 = np.exp(x3) * np.exp(x2*lr/lv) # INTERNE DYNAMIK

        Z = np.array([z1, z2]).reshape(2,1)
        alpha = v**2*sin(x3)*cos(x2)/lr
        beta = -lv*v*cos(x2)/lr


        
        # Calculate the Control
        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            print("Ts = ", Ts, self.sim_param.Ts_sim)
            A = np.array( [[1, Ts], [0, 1] ] ).reshape(2,2)
            B = np.array( [0, Ts] ).reshape(2,1)
            Q = np.array( [[10, 0], [0, 10]] )
            R = np.array( [1.0] ).reshape(1,1)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
        ctrl_linear = -self.ctrlMatrix_K @ Z
        ctrl_linear = ctrl_linear[0,0]

        ctrl = (-alpha + ctrl_linear) / beta # See "Jürgen Adamy Nichtlineare Regelungen" p.338

        v**2*sin(x3)*cos(x2)/lr 

        self.steer += Ts*ctrl
        print("steer: ",self.steer)




        return self.steer

    def _skidsteer__u_deltaV__n2(self, diff_pose, velocity):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        b = self.robot_param.wheelbase

        # GET THE REFERENCE-POSE
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)       

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        Z = np.array([z1, z2]).reshape(2,1)

        # print("VELOCITY:", v)
        if v == 0.0: 
            return 0.0
        beta = np.array( [ v * cos(x2) / b]).reshape(1,1)
        beta = np.asscalar(beta)
        alpha = np.array( [0.0] ).reshape(1,1)
        alpha = np.asscalar(alpha)

        ctrl_linear = -self.get_ctrlMtrx_K(n=2, m=1) @ Z
        ctrl_linear = np.asscalar(ctrl_linear)
        ctrl = (-alpha + ctrl_linear) / beta # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        return ctrl

    def _skidsteer__u_deltaV__n3(self, diff_pose, velocity):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        b = self.robot_param.wheelbase

        # GET THE REFERENCE-POSE
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)            
        x3 = self.dVel

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (x3*v*cos(x2))/b
        Z = np.array([z1, z2, z3]).reshape(-1,1)

        if v == 0.0: 
            return 0.0
        beta = np.array( [ (v*cos(x2))/b]).reshape(-1,1)
        try:
            alpha = np.array( [-(x3**2*v*sin(x2))/b**2] ).reshape(-1,1)
        except Exception as e:
            print(e)
            alpha = np.array([0])

        beta = np.asscalar(beta)
        alpha = np.asscalar(alpha)

        # if self.ctrlMatrix_K is None:
        #     Ts = self.sim_param.Ts_ctrl
        #     A = np.array( [[1, Ts, 0], [0, 1, Ts], [0, 0, 1] ] )
        #     B = np.array( [0, 0, Ts] ).reshape(3, 1)
        #     Q = np.array( [[self.q_yE, 0, 0], [0, self.q_yawE, 0], [0, 0, self.q_steer]] )
        #     R = np.array( [self.q_steerRate] ).reshape(1, 1)
        #     self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
        if self.ctrlMatrix_K is None:
            return 0
        ctrl_linear = -self.ctrlMatrix_K @ Z
        ctrl_linear = np.asscalar(ctrl_linear)
        ctrl = (-alpha + ctrl_linear) / beta # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # ctrl = np.asscalar(ctrl)
        self.dVel += Ts * ctrl 
        return ctrl

    def _ackermann_frontSteering_RearDrive__u_steerRate__n3(self, diff_pose, velocity, steer, T_steer=None):
        v = velocity
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase

        # GET THE REFERENCE-POSE
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)     
        x3 = steer

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0 * cos(x2) * tan(x3) ) / lw

        Z = np.array([z1, z2, z3]).reshape(3,1)
        beta = np.array( [ (v**2.0 * cos(x2) * (tan(x3)**2.0 + 1) ) / (lw)]).reshape(1,1)

        # Calculate the Control
        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0], [0, 1, Ts], [0, 0, 1] ] )
            B = np.array( [0, 0, Ts] ).reshape(3, 1)
            Q = np.array( [[self.q_yE, 0, 0], [0, self.q_yawE, 0], [0, 0, self.q_steer]] )
            R = np.array( [self.q_steerRate] ).reshape(1, 1)
            print(Q, R)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
        # print("K.shape = ", self.ctrlMatrix_K.shape)
        # print("Z.shape = ", Z.shape)
        ctrl_linear = - self.ctrlMatrix_K @ Z
        alpha = np.array( [- (v**3.0 * sin(x2) * tan(x3)**2.0) / lw**2.0] ).reshape(1,1)
        
        # print("ctrl_linear = ", ctrl_linear)

        ctrl = (-alpha + ctrl_linear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        # print("ctrl[0,0] = ", ctrl[0,0])

        return ctrl[0,0]

    def _ackermann_frontSteering_RearDrive_n3(self, diff_pose, velocity, steer, T_steer=None):
        v = velocity
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase
        if T_steer is None:
            Tsteer = self.robot_param.steerPT1_ctrl_TL
        else:
            Tsteer = T_steer

        # GET THE REFERENCE-POSE
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)     
        x3 = steer

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0 * cos(x2) * tan(x3) ) / lw
        Z = np.array([z1, z2, z3]).reshape(3,1)
        beta = np.array( [ (v**2.0 * cos(x2) * (tan(x3)**2.0 + 1) ) / (Tsteer*lw)]).reshape(1,1)


        # print("Z = ", Z)
        
        # Calculate the Control
        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0], [0, 1, Ts], [0, 0, 1] ] )
            B = np.array( [0, 0, Ts] ).reshape(3, 1)
            Q = np.array( [[self.q_yE, 0, 0], [0, self.q_yawE, 0], [0, 0, self.q_steer]] )
            R = np.array( [self.q_steerRate] ).reshape(1, 1)
            # print(Q, R)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
        # print("K.shape = ", self.ctrlMatrix_K.shape)
        # print("Z.shape = ", Z.shape)
        ctrl_linear = - self.ctrlMatrix_K @ Z
        alpha_1 = - (v**3.0 * sin(x2) * tan(x3)**2.0) / lw**2.0

        alpha_2 =  - (v**2.0 * x3 * cos(x2) * (tan(x3)**2.0 + 1.0) ) / (Tsteer*lw)
        alpha = np.array( [alpha_1 + alpha_2] ).reshape(1,1)
        
        # print("ctrl_linear = ", ctrl_linear)

        ctrl = (-alpha + ctrl_linear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        ctrl = np.arctan2(ctrl, 1)
        # print("ctrl[0,0] = ", ctrl[0,0])

        return ctrl[0,0]

    def _ackr_errmodel__pt1__u_steerrate_n4(self, diff_pose, velocity, steer):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase


        # SET THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = steer
        x4 = self.steer
        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0*cos(x2)*tan(x3))/lw
        l_r = lw
        theta_e = x2
        delta = steer
        delta_des = self.steer
        T_L = self.robot_param.steerPT1_ctrl_TL
        z4 =  -(- T_L*sin(theta_e)*(cos(delta)**2 - 1)*v**3 + l_r*(delta*cos(theta_e) - delta_des*cos(theta_e))*v**2)/(T_L*l_r**2*cos(delta)**2)
        zStateVec = np.array([z1, z2, z3, z4]).reshape(-1,1)
        alpha =  np.array([((delta - delta_des)*((v**2*cos(theta_e)*(tan(delta)**2 + 1))/(T_L*l_r) + \
                           (2*v**3*tan(delta)*sin(theta_e)*(tan(delta)**2 + 1))/l_r**2 + \
                           (2*v**2*cos(theta_e)*tan(delta)*(delta - delta_des)*(tan(delta)**2 + 1))/(T_L*l_r)))/T_L - \
                           (v*tan(delta)*((v**3*cos(theta_e)*tan(delta)**2)/l_r**2 - \
                           (v**2*sin(theta_e)*(delta - delta_des)*(tan(delta)**2 + 1))/(T_L*l_r)))/l_r]).reshape(-1,1)
        # print("alpha:", alpha, self.steer)
        beta = np.array([(v**2*cos(theta_e))/(T_L*l_r*cos(delta)**2)]).reshape(-1,1)

        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0, 0], [0, 1, Ts, 0], [0, 0, 1, Ts], [0, 0, 0, 1]] ).reshape(4,4)
            B = np.array( [0, 0, 0, Ts] ).reshape(4,1)
            Q = np.eye(4,4)
            Q[0, 0] = self.q_yE
            Q[1, 1] = self.q_yawE
            Q[2, 2] = self.q_steer
            Q[3, 3] = self.q_steer
            R = np.array( [self.q_steerRate*0] ).reshape(1,1)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
            # print(Q, R)

        v_ctrlLinear = -self.ctrlMatrix_K @ zStateVec

        # print(alpha, v_ctrlLinear, beta)
        u_steerRate = (-alpha + v_ctrlLinear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        # ctrl = np.arctan2(ctrl, 1)
        # print("u_steerRate = ",u_steerRate[0,0])

        self.steer += Ts * u_steerRate[0,0]
        # if np.fabs(self.steer) > np.deg2rad(36.0):
            # self.steer = np.deg2rad(36.0) * np.sign(self.steer)

        return self.steer

    def _ackr_errmodel__pt2__u_steer__n4(self, diff_pose, velocity, steer, steer_rate):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase


        # SET THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = steer
        x4 = steer_rate
        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0*cos(x2)*tan(x3))/lw
        l_r = lw
        theta_e = x2
        dot_delta = steer_rate
        delta = steer
        T_1 = self.robot_param.steerPT2_ctrl_T1
        T_2 = self.robot_param.steerPT2_ctrl_T2
        z4 = (v**3*sin(theta_e))/l_r**2 - (v**2*(v*sin(theta_e) - dot_delta*l_r*cos(theta_e)))/(l_r**2*cos(delta)**2)


        zStateVec = np.array([z1, z2, z3, z4]).reshape(4,1)
        alpha =  np.array([- (v**4*sin(delta)*cos(theta_e) - v**4*cos(delta)**2*sin(delta)*cos(theta_e) - 2*dot_delta**2*l_r**2*v**2*sin(delta)*cos(theta_e) + 3*dot_delta*l_r*v**3*sin(delta)*sin(theta_e))/(l_r**3*cos(delta)**3) - (v**2*(delta*cos(theta_e) - T_1*dot_delta*cos(theta_e)))/(T_2*l_r*cos(delta)**2)]).reshape(1,1)
        beta = np.array([(v**2*cos(theta_e))/(T_2*l_r*cos(delta)**2)]).reshape(1,1)

        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0, 0], [0, 1, Ts, 0], [0, 0, 1, Ts], [0, 0, 0, 1]] ).reshape(4,4)
            B = np.array( [0, 0, 0, Ts] ).reshape(4,1)
            Q = np.eye(4,4)
            Q[0, 0] = self.q_yE
            Q[1, 1] = self.q_yawE
            Q[2, 2] = self.q_steer
            Q[3, 3] = self.q_steerRate
            # R = np.array( [self.q_steer] ).reshape(1,1)
            R = np.array( [0] ).reshape(1,1)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
            # print(Q, R)   

        v_ctrlLinear = -self.ctrlMatrix_K @ zStateVec

        # print(alpha, v_ctrlLinear, beta)
        u_steer = (-alpha + v_ctrlLinear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        # ctrl = np.arctan2(ctrl, 1)
        # print("u_steerRate = ",u_steerRate[0,0])

        self.steer = u_steer[0,0]
        return self.steer

    def _ackr_errmodel__pt2__u_steerrate(self, diff_pose, velocity, steer, steer_rate):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase


        # SET THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = steer
        x4 = steer_rate
        x5 = self.steer
        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0*cos(x2)*tan(x3))/lw
        l_r = lw
        theta_e = x2
        dot_delta = steer_rate
        delta = steer
        delta_des = self.steer
        T_1 = self.robot_param.steerPT2_lqr_T1
        T_2 = self.robot_param.steerPT2_lqr_T2
        z4 = (v**3*sin(theta_e))/l_r**2 - (v**2*(v*sin(theta_e) - dot_delta*l_r*cos(theta_e)))/(l_r**2*cos(delta)**2)
        z5 = - (v**3*sin(delta)*(v*cos(theta_e) + dot_delta*l_r*sin(theta_e) - v*cos(delta)**2*cos(theta_e)))/(l_r**3*cos(delta)**3) - (v**2*cos(theta_e)*(delta - delta_des + T_1*dot_delta))/(T_2*l_r*cos(delta)**2) - (2*dot_delta*v**2*sin(delta)*(v*sin(theta_e) - dot_delta*l_r*cos(theta_e)))/(l_r**2*cos(delta)**3)

        zStateVec = np.array([z1, z2, z3, z4, z5]).reshape(5,1)
        alpha =  np.array( [(v**2*(T_1**2*dot_delta*l_r**3*cos(delta)**2*cos(theta_e) + T_1*delta*l_r**3*cos(delta)**2*cos(theta_e) - T_1*delta_des*l_r**3*cos(delta)**2*cos(theta_e)) - T_2*v**2*(dot_delta*l_r**3*cos(delta)**2*cos(theta_e) + 6*delta*dot_delta*l_r**3*cos(delta)*sin(delta)*cos(theta_e) - 6*delta_des*dot_delta*l_r**3*cos(delta)*sin(delta)*cos(theta_e) - 4*delta*l_r**2*v*cos(delta)*sin(delta)*sin(theta_e) + 4*delta_des*l_r**2*v*cos(delta)*sin(delta)*sin(theta_e) + 6*T_1*dot_delta**2*l_r**3*cos(delta)*sin(delta)*cos(theta_e) - 4*T_1*dot_delta*l_r**2*v*cos(delta)*sin(delta)*sin(theta_e)))/(T_2**2*l_r**4*cos(delta)**4) + (v**2*(v**3*sin(theta_e) - 2*v**3*cos(delta)**2*sin(theta_e) + v**3*cos(delta)**4*sin(theta_e) + 6*dot_delta**3*l_r**3*cos(theta_e) - 11*dot_delta**2*l_r**2*v*sin(theta_e) - 6*dot_delta*l_r*v**2*cos(theta_e) - 4*dot_delta**3*l_r**3*cos(delta)**2*cos(theta_e) + 6*dot_delta*l_r*v**2*cos(delta)**2*cos(theta_e) + 8*dot_delta**2*l_r**2*v*cos(delta)**2*sin(theta_e)))/(l_r**4*cos(delta)**4)] ).reshape(1,1)
        beta = np.array([(v**2*cos(theta_e))/(T_2*l_r*cos(delta)**2)]).reshape(1,1)
        
        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0, 0, 0], [0, 1, Ts, 0, 0], [0, 0, 1, Ts, 0], [0, 0, 0, 1, Ts], [0, 0, 0, 0, 1]] ).reshape(5,5)
            B = np.array( [0, 0, 0, 0, Ts] ).reshape(5,1)
            Q = np.eye(5, 5)
            Q[0, 0] = self.q_yE
            Q[1, 1] = self.q_yawE
            Q[2, 2] = self.q_steer
            Q[3, 3] = 0
            Q[4, 4] = 0
            R = np.array( [self.q_steerRate] ).reshape(1,1)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
            print(Q, R)

        v_ctrlLinear = -self.ctrlMatrix_K @ zStateVec

        # print(alpha, v_ctrlLinear, beta)
        u_steerRate = (-alpha + v_ctrlLinear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        # ctrl = np.arctan2(ctrl, 1)
        # print("u_steerRate = ",u_steerRate[0,0])

        self.steer += Ts * u_steerRate[0,0]

        return self.steer

    def _ackr_errmodel_frontsteer_velRear__u_steerRate__n3(self, diff_pose, velocity, steer):
        v = velocity
        # print(v)
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase

        # print(lo, lw, Ts)

        # SET THE ERROR-STATE-VECTOR
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = steer
        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        z3 = (v**2.0*cos(x2)*tan(x3))/lw
        zStateVec = np.array([z1, z2, z3]).reshape(3,1)
        alpha =  np.array( -(v**3.0*tan(x3)**2.0*sin(x2))/lw**2.0 ).reshape(1,1)
        beta = np.array( (v**2.0*cos(x2)*(tan(x3)**2.0 + 1))/lw ).reshape(1,1)

        if self.ctrlMatrix_K is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts, 0], [0, 1, Ts], [0, 0, 1]] ).reshape(3,3)
            B = np.array( [0, 0, Ts] ).reshape(3,1)
            Q = np.eye(3, 3)
            Q[0, 0] = self.q_yE
            Q[1, 1] = self.q_yawE
            Q[2, 2] = self.q_steer
            R = np.array( [self.q_steerRate] ).reshape(1,1)
            self.ctrlMatrix_K = self._getDLQR(A, B, Q, R)
            # print(Q, R)

        v_ctrlLinear = -self.ctrlMatrix_K @ zStateVec

        # print(alpha, v_ctrlLinear, beta)
        u_steerRate = (-alpha + v_ctrlLinear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        # print("ctrl.shape = ", ctrl.shape)
        # ctrl = np.arctan2(ctrl, 1)
        # print("u_steerRate = ",u_steerRate[0,0])

        self.steer += Ts * u_steerRate[0,0]

        return self.steer


    def _ackermann_frontSteering_RearDrive_n2(self, diff_pose, velocity):
        v = velocity
        Ts = self.sim_param.Ts_ctrl
        lo = self.robot_param.length_offset
        lw = self.robot_param.wheelbase

        # DEFINE THE ERROR-STATE-VECTOR
        if type(diff_pose) == type(np.array([])):
            x1 = diff_pose[1, 3]
            x2 = matrix_to_yaw(diff_pose)
        elif type(diff_pose) == type([]):
            x1 = diff_pose[0]
            x2 = diff_pose[1]
        else:
            x1 = diff_pose["yError"]
            x2 = diff_pose["yawErr"]

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = v*sin(x2)
        Z = np.array([z1, z2]).reshape(2,1)
        alpha = np.array( [0] ).reshape(1,1)
        beta = np.array( [ v**2.0 * cos(x2) / lw]).reshape(1,1)

        # print(self.get_ctrlMtrx_K(n=2, m=1).shape)

        ctrl_linear = - self.get_ctrlMtrx_K(n=2, m=1) @ Z
        ctrl = (-alpha + ctrl_linear) @ inv(beta) # See "Jürgen Adamy Nichtlineare Regelungen" p.338
        ctrl = np.arctan2(ctrl, 1)

        return ctrl[0,0]

    def _ackermann_deltaV_vH__u_steer_accel__n2(self, diff_pose):
        # GET PARAMETERS
        lo = self.robot_param.length_offset
        lr = self.robot_param.wheelbase
        Ts = self.sim_param.Ts_ctrl

        # GET STATES
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)
        x3 = self.velocity

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = x3*sin(x2)
        z3 = x3
        Z = np.array([z1, z2, z3]).reshape(3,1)

        # Calculate the LINEAR Control
        if self.ctrlMatrix_K_yE is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [[1, Ts], [0, 1]] )
            B = np.array( [0, Ts] ).reshape(2,1)            
            Q = np.array( [[100, 0], [0, 100]] )
            R = np.array( [1.0] ).reshape(1,1)

            self.ctrlMatrix_K_yE = np.array(self._getDLQR(A, B, Q, R))
            A = np.array( [1] ).reshape(1,1)
            B = np.array( [Ts] ).reshape(1,1)
            Q = np.array( [1]  ).reshape(1,1)
            R = np.array( [1.0] ).reshape(1,1)
            self.ctrlMatrix_K_thetaE = np.array(self._getDLQR(A, B, Q, R))

        # print(self.ctrlMatrix_K, z1, z2)
        v1 = -self.ctrlMatrix_K_yE @ np.array([z1, z2]).reshape(2,1)
        v2 = -self.ctrlMatrix_K_thetaE @ np.array(z3).reshape(1,1)
        v_linear = np.array( [v1, v2]).reshape(2,1)

        # LINEAR TO NONLINEAR CONTROL
        D_inv =  np.array([ (0, 1),
                            (lr/(x3**2.0 * cos(x2)), -(lr*sin(x2))/(x3**2.0*cos(x2))) ])

        c_hat = np.array([ (0, 0) ]).reshape(2,1)
        # print(D_inv.shape, c_hat.shape, v_linear.shape)
        u = -D_inv @ (c_hat - v_linear)
        print("u:", u.T)
        u1 = u[0, 0]
        u2 = u[1, 0]

        print("velocity:", self.velocity)

        steer = np.arctan2(u2, u1)
        print("steer:", np.rad2deg(steer) )

        self.velocity += Ts*u1

        return steer, self.velocity
    #########################################################




    def _ackermann_deltaV_vH_u2_n2(self, diff_pose):
        # GET PARAMETERS
        lo = self.robot_param.length_offset
        lr = self.robot_param.wheelbase

        # GET STATES
        x1 = diff_pose[1, 3]
        x2 = matrix_to_yaw(diff_pose)        

        # STATE TRANSFORMATION
        z1 = x1 - lo*sin(x2)
        z2 = x2
        Z = np.array([z1, z2]).reshape(2,1)

        # Calculate the LINEAR Control
        if self.ctrlMatrix_K_yE is None:
            Ts = self.sim_param.Ts_ctrl
            A = np.array( [1] ).reshape(1,1)
            B = np.array( [Ts] ).reshape(1,1)
            Q = np.array( [1e5]  ).reshape(1,1)
            R = np.array( [1.0] ).reshape(1,1)
            self.ctrlMatrix_K_yE = np.array(self._getDLQR(A, B, Q, R))

            Q = np.array( [1e1]  ).reshape(1,1)
            self.ctrlMatrix_K_thetaE = np.array(self._getDLQR(A, B, Q, R))

        # print(self.ctrlMatrix_K, z1, z2)
        v1 = -self.ctrlMatrix_K_yE @ np.array(z1).reshape(1,1)
        v2 = -self.ctrlMatrix_K_thetaE @ np.array(z2).reshape(1,1)
        v_linear = np.array( [v1, v2]).reshape(2,1)

        # LINEAR TO NONLINEAR CONTROL
        D_inv =  np.array([ (1/sin(x2),  0),
                            (0,              lr) ])




        c_hat = np.zeros( shape=(2,1))
        # print(D_inv.shape, c_hat.shape, v_linear.shape)
        u = -D_inv @ (c_hat - v_linear)
        print("u:", u.T)
        u1 = u[0, 0]
        u2 = u[1, 0]

        velocity = u1
        print("velocity:", velocity)

        steer = np.arctan2(u2, u1)
        print("steer:", np.rad2deg(steer) )

        return steer, velocity
