# PYTHON
import numpy as np
from numpy import sin, cos, tan
import helpers.transformation_helper as tf

class Robot_State() :
    def __init__(self, robot_parameter, x=0.0, y=0.0, yaw=0.0, steerAngle=0.0,
                 velocity=0.0, var_xy=0.0, var_yaw=0.0, l10n_update_distance=0.0,
                 wheelloader_type="no_wheelloader", time_delay=None) :
    
        self.wheelloader_type = wheelloader_type
        self.look_ahead = robot_parameter.look_ahead
        self.length_offset = robot_parameter.length_offset
        self.var_xy = var_xy
        self.var_yaw = var_yaw
        self.l10n_update_distance = l10n_update_distance
        
        if time_delay is not None:
            self.oTimeDelay =  TimeDelay(Ts=time_delay[0], time_delay_=time_delay[1], init_xyYaw=None)
            self.time_delayed = True
        else:
            self.time_delayed = False

        # self.init = Init(x=x, y=y, yaw=yaw, steer_angle=steerAngle, velocity=velocity)
        self.reset_me(x, y, yaw, steerAngle, velocity)

    def update(self, x, y, yaw, velocity, steerAngle=0.0, steerRate=0.0):
        self.x = x
        self.y = x
        self.yaw = yaw
        self.velocity = velocity
        self.steer = steerAngle
    def get_xPosition(self):
        return self.x()

    def as_dict(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "yawRate": self.yawRate,
                "steer": self.steerAngle, "steerRate": self.steerAngleRate, "v": self.velocity,
                "xOffset": self.xo, "yOffset": self.yo, "deltaVel": self.deltaVel, 
                "vRight": self.vR, "vLeft": self.vL, "yawRate_front": self.yawRate_front, "yawRate_rear": self.yawRate_rear,
                "yaw_front": self.yaw_front, "yaw_rear": self.yaw_rear, "steer_front": self.steerAngle}
    #### OLD FROM DISS
        
    def update_lengthoffset(self, length_offset):
        self.length_offset = length_offset

    def reset_me(self, x=None, y=None, yaw=None, steerAngle=None, velocity=None):
        # print("ROBOTPOSES INIT:", self.init)
        if x is None: x = self.init.x
        if y is None: y = self.init.y
        if yaw is None: yaw = self.init.yaw
        if steerAngle is None: steerAngle = self.init.steer_angle
        if velocity is None: velocity = self.init.velocity

        self.xyYaw_update = None
        
        # POSE
        self.x = x
        self.y = y
        self.xo = x + self.length_offset * cos(yaw)
        self.yo = y + self.length_offset * sin(yaw)
        self.yaw = yaw

        if velocity > 0.0:
            self.yaw_front = self.yaw
            self.yaw_rear = self.yaw_front - steerAngle
            self.yaw_rear = tf.minusPi_to_pi(self.yaw_rear)
        else:
            # ic(self.yaw)
            self.yaw_rear = self.yaw
            self.yaw_front = self.yaw_rear + steerAngle
            self.yaw_front = tf.minusPi_to_pi(self.yaw_front)

        self.yawRate = 0.0
        self.yawRate_front = 0.0
        self.yawRate_rear = 0.0

        self.xr = x
        self.yr = y
        self.xf = 0.0
        self.yf = 0.0

        # Localization
        self.lx = x
        self.ly = y
        self.lYaw = yaw
        self.lxo = self.xo
        self.lyo = self.yo
        # STEERING
        self.steerAngle = steerAngle
        self.steerAngleDes = 0.0
        self.steerAngleRate = 0.0
        self.steerAngleRateDes = 0.0

        self.deltaVel = 0.0
        self.deltaVelDes = 0.0
        self.vR = velocity
        self.vL = velocity

        # DRIVE  
        self.velocity = velocity
        self.accel = 0.0


        if self.time_delayed == True:
            self.oTimeDelay =  TimeDelay(Ts=self.oTimeDelay.Ts,
                                         time_delay_=self.oTimeDelay.time_delay,
                                         init_xyYaw=None)


        # return

    ########################
    ##### SET METHODS ######
    def set_velocity(self, vel, diffVel_skid=None, vel_right=None, vel_left=None) :
        # print("ROBOT STATE set_velocity", vel, diffVel_skid, vel_right, vel_left, "|||||", self.velocity, self.deltaVel, self.vR, self.vL)
        with self._mutex:
            self.velocity = vel
            self.deltaVel = diffVel_skid
            self.vR = vel_right
            self.vL = vel_left
    def set_pose(self, x, y, yaw, yawRate = None, simtime=None, yaw_front=None, yaw_rear=None, yawRate_front=None, yawRate_rear=None) :

        if self.time_delayed==True and self.oTimeDelay.values is None:
            self.oTimeDelay =  TimeDelay(Ts=self.oTimeDelay.Ts,
                                         time_delay_=self.oTimeDelay.time_delay,
                                         init_xyYaw=[x, y, yaw])

        # print("haha", self.yaw_rear)
        with self._mutex:
            self.x = x
            self.y = y
            self.yaw = yaw
        
            if self.velocity > 0.0:
                if yaw_front is None:
                    self.yaw_front = self.yaw
                    self.yaw_rear = self.yaw_front - self.get_steerangle()
                    self.yaw_rear = tf.minusPi_to_pi(self.yaw_rear)
                else:
                    self.yaw_front = yaw_front
                    self.yaw_rear = yaw_rear
            else:
                if yaw_rear is None:
                    self.yaw_rear = self.yaw
                    self.yaw_front = self.yaw_rear + self.get_steerangle()
                    self.yaw_front = tf.minusPi_to_pi(self.yaw_front)
                else:
                    self.yaw_front = yaw_front
                    self.yaw_rear = yaw_rear

            # print("self.yaw_front, self.yaw_rear:", self.yaw_front, self.yaw_rear)

            # print("Robot_State::set_pose(...): yaw = ", yaw)
            self.xo = x + self.length_offset * cos(yaw)
            self.yo = y + self.length_offset * sin(yaw)
            # if self.length_offset > 0.0: 
                # print("#######################################")
            # print(x, y, self.xo ,self.yo, self.length_offset)
            # ic(self.xo, self.yo)
            if yawRate is not None:
                self.yawRate = yawRate

            if yawRate_front is not None:
                self.yawRate_front = yawRate_front
            if yawRate_rear is not None:
                self.yawRate_rear = yawRate_rear

            # self.yaw_front = yaw_front
            # self.yaw_rear = yaw_rear

        self._update_l10n(yawRate, simtime)
    def set_offsetPosition(self, xo, yo) :
        with self._mutex:
            self.xo = xo
            self.yo = yo
    def set_steerAngle(self, steerAngle, steerAngleRate=None):
        with self._mutex:
            self.steerAngle = steerAngle
            # if np.fabs(np.rad2deg(steerAngle)) > 35.1: 
                # print("ROBOT STATE: STEER  OVER 35 GRAD", np.rad2deg(steerAngle))
            # print("asd") 
            if steerAngleRate is not None:
                self.steerAngleRate = steerAngleRate

    ########################
    ##### GET METHODS ######
    def get_steerangle(self):
        return self.steerAngle
    def get_steerRate(self):
        return self.steerAngleRate
    def get_asVector(self):
        with self._mutex:
            return [self.x, self.y, self.yaw, self.steerAngle, self.xo, self.yo, self.velocity, self.steerAngleRate, self.yawRate] 
            #          0       1        2          3               4        5        6                  7                   8
    def get_yaw(self):
        with self._mutex:
            return self.yaw
    def get_xyYaw(self):
        with self._mutex:
            # print(self.length_offset)
            return self.x, self.y, self.yaw
    def get_xoYoYaw(self):
        with self._mutex:
            return self.xo, self.yo, self.yaw
    def get_asVector4lookahead(self):
        with self._mutex:
            lh = self.look_ahead
            x = self.x + lh * cos(self.yaw)
            y = self.y + lh * sin(self.yaw)
            return [x, y, self.yaw] 
    def get_asDict(self):#
        # ic(self.yaw_front, self.yaw_rear)
        with self._mutex:
            return {"x": self.x, "y": self.y, "yaw": self.yaw, "yawRate": self.yawRate,
                       "steer": self.steerAngle, "steerRate": self.steerAngleRate, "velocity": self.velocity,
                       "xOffset": self.xo, "yOffset": self.yo, "deltaVel": self.deltaVel, 
                       "velRight": self.vR, "velLeft": self.vL, "yawRate_front": self.yawRate_front, "yawRate_rear": self.yawRate_rear,
                       "yaw_front": self.yaw_front, "yaw_rear": self.yaw_rear}
    def get_velocity(self):
        with self._mutex:
            return self.velocity
    def get_skid_velocities(self):
        with self._mutex:
            return self.velocity, self.deltaVel, self.vR, self.vL            
    def get_offsetAsVector(self):
        with self._mutex:
            return [self.xo, self.yo, self.yaw] 
    ########################

    ##########################
    ###### LOCALIZATION ######
    def _update_l10n(self, yawRate, simtime):
        # print("simtime:", simtime)

        if self.l10n_update_distance >=1e-6 and (self.var_xy > 1e-6 or self.var_yaw > 1e-6) :
            self._l10n_discontinous(yawRate)
            # self._set_l10n_pose(self.x, self.y, self.yaw)
        elif self.var_xy > 1e-6 or self.var_yaw > 1e-6:
            self._l10n_continous()
        elif self.time_delayed == True and simtime is not None:
            # print("TIME DELAY _update_l10n", (self.x, self.y, self.yaw), simtime)
            x, y, yaw = self.oTimeDelay.getTimeDelayed(simtime, np.array([self.x, self.y, self.yaw]) )
            # print(self.x, x)
            self._set_l10n_pose(x, y, yaw)
        else:
            # print("_update_l10n in ELSE:")
            self._set_l10n_pose(self.x, self.y, self.yaw)

    def _l10n_continous(self):
        x, y, yaw = self.get_xyYaw()
        dx = np.random.normal(0.0, self.var_xy)
        dy = np.random.normal(0.0, self.var_xy)
        dYaw = np.random.normal(0.0, self.var_yaw)
        lx = x + dx
        ly = y + dy
        lYaw = tf.getYawRightRange(yaw + dYaw)
        # print("x, y, yaw:", x, y, yaw, "; lx, ly, lYaw:", lx, ly, lYaw)
        self._set_l10n_pose(lx, ly, lYaw)
    def _l10n_discontinous(self, yawRate):
        xyYaw_now = self.get_xyYaw()

        tfMatrix_now = tf.xyYaw_to_matrix(xyYaw_now)

        if self.xyYaw_update is None:
            self.xyYaw_update = xyYaw_now
            

        if tf.getLength2D(xyYaw_now, self.xyYaw_update) > self.l10n_update_distance:
            self.xyYaw_update = xyYaw_now
            dx = np.random.normal(0.0, self.var_xy)
            dy = np.random.normal(0.0, self.var_xy)
            dYaw = np.random.normal(0.0, self.var_yaw)
            dYaw = tf.getYawRightRange(dYaw)
            self.dtfMatrix = tf.xyYaw_to_matrix(dx, dy, dYaw)


        map_2_l10n = tfMatrix_now @ self.dtfMatrix
        lx, ly, lYaw = map_2_l10n[0,3], map_2_l10n[1,3], tf.matrix_to_yaw(map_2_l10n)
        self._set_l10n_pose(lx, ly, lYaw)


    def _set_l10n_pose(self, lx, ly, lYaw):
        with self._mutex:
            self.lx = lx
            self.ly = ly
            self.lYaw = lYaw
            # print("self.length_offset:", self.length_offset)
            self.lxo = lx + self.length_offset * cos(lYaw)
            # print("lx:", lx, "lxo:", self.lxo)
            self.lyo = ly + self.length_offset * sin(lYaw)
    def get_l10nVec(self):
        with self._mutex:
            return [self.lx, self.ly, self.lYaw, self.steerAngle, self.lxo, self.lyo, self.velocity, self.steerAngleRate, self.yawRate, self.xr, self.yr, self.xf, self.yf] 
            #          0       1        2          3               4        5        6                  7                   8
    def get_l10nDict(self):
        with self._mutex:
            return {"lx": self.lx, "ly": self.ly, "lYaw": self.lYaw, "yawRate": self.yawRate,
                       "steer": self.steerAngle, "steerRate": self.steerAngleRate, "velocity": self.velocity,
                       "lxOffset": self.lxo, "lyOffset": self.lyo}
    def get_l10nOffset(self):
        with self._mutex:
            return [self.lxo, self.lyo, self.lYaw, self.steerAngle, self.velocity] 


    def get_wheelloader_offset_pose(self):
        if self.wheelloader_type == "wheelloader_front":
            return [self.lxo, self.lyo, self.lYaw] 
        elif self.wheelloader_type == "wheelloader_rear":
            return [self.lxo, self.lyo, self.lYaw] 
    ##################################

    ##################
    ##### PRINT ######
    def __str__(self):
        return "x, y, yaw[°], steer[°]: " + str(self.x) + " " + str(self.y) + " " + str( np.rad2deg(self.yaw))  + " " + str( np.rad2deg(self.steerAngle))
    ##################




#################### TIME DELAY FOR FENDT #########################
class TimeDelay():
    def __init__(self, Ts, time_delay_, init_xyYaw):
        self.Ts = Ts
        self.time_delay = time_delay_ # 1/0.04s = 25Hz

        self.values = init_xyYaw
        self.shiftVec = np.array( [] )

    def getTimeDelayed(self, time, xyYaw):
        # print("#######################")
        time = round(time, 6)

        # self.shiftVec = np.append(self.shiftVec, [time+self.time_delay, xyYaw] ).reshape(-1,4)
        # self.shiftVec = np.vstack( (self.shiftVec, [time+self.time_delay, xyYaw]) ).reshape(-1,4)
        new_entry = np.hstack(([time+self.time_delay, xyYaw])).reshape(1,-1)
        # print(new_entry.shape)
        self.shiftVec = np.vstack( (self.shiftVec, new_entry) ) if self.shiftVec.size > 0 else new_entry

        # print("NUN:_", self.shiftVec.shape)


        # if time > self.shiftVec[0,0]:
        while time > self.shiftVec[0,0]:
            # print("while:", self.shiftVec.shape)
            self.values = self.shiftVec[0,1:]
            # print("before shift:", self.shiftVec.shape, self.shiftVec[1: , :])
            self.shiftVec = self.shiftVec[1:, :]
            # self.shiftVec = np.delete(self.shiftVec, 1, 0 )
            # print("after shift:", self.shiftVec.shape)

        # print("self.shiftVec:", self.shiftVec, " @time:", time)
        # return steer
        return self.values





if __name__ == "__main__":
    print("MAIN for testing the class")

    Ts = 0.01
    T_Dead = 0.1
    rp = Robot_Parameter(length_offset=0.0)
    rs = Robot_State(rp, time_delay=(Ts, T_Dead))

    v = 0.5
    lw = 3.0

    x = 0.0
    y = 0.0
    yaw = 0.0

    X = [x]
    Y = [y]
    D = [yaw]
    T = [0.0]
    L = []

    for i in range(1000):
        print("\niter:", i)
        delta = np.sin(0.01*i) * 0.6
        dx = v * np.cos(yaw)
        dy = v * np.sin(yaw)
        dyaw = v/lw * np.tan(delta)

        x += Ts * dx
        y += Ts * dy
        yaw += Ts * dyaw
        X.append(x)
        Y.append(y)
        D.append(delta)
        T.append(T[-1]+Ts)
        rs.set_pose(x, y, yaw, simtime=i*Ts)
        l10n = rs.get_l10nOffset() # [self.lxo, self.lyo, self.lYaw, self.steerAngle, self.velocity] 
        L.append(l10n[0:2])
        # print(len(L))
        
    L = np.array(L)
    print("np:", type(L), L.shape)
    # print("??:", L[:,0])

    import matplotlib.pyplot as plt
    # plt.plot(X, Y, '--')
    # plt.plot(L[:,0], L[:,1])
    # plt.plot(T, D)

    plt.plot(X)
    plt.plot(L[:,0])
    plt.show()

