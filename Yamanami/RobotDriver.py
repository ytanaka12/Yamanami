from typing import Tuple, Union
from Yamanami.ODriveDriver import ODriveDriver
from Yamanami.myutil import *
import time


class RobotDriver(ODriveDriver):

    # for zero stance
    ZeroPointOffsetMotorAngle_0 = 0.0 * DEG2RAD  # rad
    ZeroPointOffsetMotorAngle_1 = 0.0 * DEG2RAD

    CAMERA_POS_Z = 0.092    #(m)

    def init(self):
        self.connectToODrive()
        self.clearError()

        self.encoderOffsetCalibration_Axis0()
        self.encoderOffsetCalibration_Axis1()

        self.calibZeroPoint_Axis0()
        self.calibZeroPoint_Axis1()
        return


    def terminate(self):
        self.moveHome()
        self.setServoOff_Axis0()
        self.setServoOff_Axis1()
        return


    def calibZeroPoint_Axis0(self):
        des_angle = self.getCurrentMotorAngle_Axis0()

        self.setServoOn_Axis0()
        while True:
            cur_angle = self.getCurrentMotorAngle_Axis0()
            err_angle = des_angle - cur_angle

            threshold = 10.0 * DEG2RAD
            if threshold < abs(err_angle):
                self.ZeroPointOffsetAngle_0 = -90.0 * DEG2RAD + cur_angle
                break

            des_angle += 0.5 * DEG2RAD
            self.setMotorAngle_Axis0(des_angle)
            time.sleep(0.02)
            pass

        self.setServoOff_Axis0()
        return


    def calibZeroPoint_Axis1(self):
        des_angle = self.getCurrentMotorAngle_Axis1()

        self.setServoOn_Axis1()
        while True:
            cur_angle = self.getCurrentMotorAngle_Axis1()
            err_angle = des_angle - cur_angle

            threshold = 10.0 * DEG2RAD
            if threshold < abs(err_angle):
                self.ZeroPointOffsetAngle_1 = -60.0 * DEG2RAD + cur_angle
                break

            des_angle += 0.5 * DEG2RAD
            self.setMotorAngle_Axis1(des_angle)
            time.sleep(0.02)
            pass

        self.setServoOff_Axis1()
        return


    def setJointAngle_Axis0(self, angle: float = 0.0):
        self.setMotorAngle_Axis0(- angle + self.ZeroPointOffsetAngle_0)
        return


    def setJointAngle_Axis1(self, angle: float = 0.0):
        self.setMotorAngle_Axis1(- angle + self.ZeroPointOffsetAngle_1)
        return


    def getCurrentJointAngle_Axis0(self):
        cur_angle = - self.getCurrentMotorAngle_Axis0() + self.ZeroPointOffsetAngle_0
        return cur_angle


    def getCurrentJointAngle_Axis1(self):
        cur_angle = - self.getCurrentMotorAngle_Axis1() + self.ZeroPointOffsetAngle_1
        return cur_angle



    def inverseKinematics(self, target_x: float, target_y: float, target_z: float) -> Tuple[float, float]:
        if 0.05 < distance3D(0.0, 0.0, 0.0, target_x, target_y, target_z):
            print("Warning: Target position is too close. Inverse Kinematics function returns current joint angles.")
            return self.getCurrentJointAngle_Axis0(), self.getCurrentJointAngle_Axis1()

        joint_angle_0 = math.atan2(target_y, target_x)
        joint_angle_1 = math.atan2(target_z - self.CAMERA_POS_Z, distance2D(0.0, 0.0, target_x, target_y))
        return joint_angle_0, joint_angle_1


    def moveJoint(self, target_angle_0=0.0, target_angle_1=0.0, time_to_move=10.0):
        cur_angle_0 = self.getCurrentJointAngle_Axis0()
        cur_angle_1 = self.getCurrentJointAngle_Axis1()
        traj_0 = TrajGen_HigherOrderPolynomials(
            cur_angle_0, target_angle_0, time_to_move
        )
        traj_1 = TrajGen_HigherOrderPolynomials(
            cur_angle_1, target_angle_1, time_to_move
        )

        tk = TimeKeeper(0.01)
        start_time = time.time()
        while True:
            t = time.time() - start_time
            if time_to_move < t:
                break

            des_angle_0 = traj_0.Get_Trajectory(t)
            des_angle_1 = traj_1.Get_Trajectory(t)

            self.setJointAngle_Axis0(des_angle_0)
            self.setJointAngle_Axis1(des_angle_1)
            tk.SleepToKeep()
        return
    
    
    def moveHome(self):
        # Stand by
        self.moveJoint(
            target_angle_0=0.0 * DEG2RAD,
            target_angle_1=0.0 * DEG2RAD,
            time_to_move=1.0,
        )
        return
    
    pass



class TrajGen_HigherOrderPolynomials:
    theta_0 = None
    theta_f = None
    theta_0_dot = 0.0
    theta_f_dot = 0.0
    theta_0_dotdot = 0.0
    theta_f_dotdot = 0.0
    t_f = None

    a0 = None
    a1 = None
    a2 = None
    a3 = None
    a4 = None
    a5 = None

    def __init__(self, start_pos=0.0, end_pos=0.0, time_to_move=10.0):
        self.Set_Conditions(start_pos, end_pos, time_to_move)
        return

    def Set_Conditions(self, start_pos=0.0, end_pos=0.0, time_to_move=10.0):
        self.theta_0 = start_pos
        self.theta_f = end_pos
        if time_to_move < 0.1:
            self.t_f = 0.1
        else:
            self.t_f = time_to_move

        self.a0 = self.theta_0
        self.a1 = self.theta_0_dot
        self.a2 = self.theta_0_dotdot / 2.0
        self.a3 = (
            20.0 * self.theta_f
            - 20.0 * self.theta_0
            - (8.0 * self.theta_f_dot + 12.0 * self.theta_0_dot) * self.t_f
            - (3.0 * self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)
        ) / (2.0 * (self.t_f ** 3.0))
        self.a4 = (
            30.0 * self.theta_0
            - 30.0 * self.theta_f
            + (14.0 * self.theta_f_dot + 16.0 * self.theta_0_dot) * self.t_f
            + (3.0 * self.theta_0_dotdot - 2.0 * self.theta_f_dotdot)
            * (self.t_f ** 2.0)
        ) / (2.0 * (self.t_f ** 4.0))
        self.a5 = (
            12.0 * self.theta_f
            - 12.0 * self.theta_0
            - (6.0 * self.theta_f_dot + 6.0 * self.theta_0_dot) * self.t_f
            - (self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)
        ) / (2.0 * (self.t_f ** 5.0))
        return

    def Get_Trajectory(self, time=0.0):
        t = time
        if self.t_f < t:
            t = self.t_f

        des_pos = self.a0
        des_pos += self.a1 * t
        des_pos += self.a2 * (t ** 2.0)
        des_pos += self.a3 * (t ** 3.0)
        des_pos += self.a4 * (t ** 4.0)
        des_pos += self.a5 * (t ** 5.0)

        return des_pos

    def Get_t_f(self):
        return self.t_f

