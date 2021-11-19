from odrive.enums import MOTOR_ERROR_ADC_FAILED
from Yamanami.ODriveDriver import ODriveDriver
from Yamanami.myutil import *
import time


class RobotDriver(ODriveDriver):

    # for zero stance of rinta
    # You must adjust below two offset angle by your hands before the system runs.
    ZeroStanceOffsetAngle_0 = 0.0 * DEG2RAD  # rad
    ZeroStanceOffsetAngle_1 = 0.0 * DEG2RAD

    def init(self):
        self.connectToODrive()
        self.clearError()

        self.encoderOffsetCalibration_Axis0()
        self.encoderOffsetCalibration_Axis1()

        self.calib_ZeroStance_Axis0()
        self.calib_ZeroStance_Axis1()

        self.set_CLOSED_LOOP_CONTROL_BothAxis()
        return


    def terminate(self):
        self.moveHome()
        self.set_IDLE_Axis0()
        self.set_IDLE_Axis1()
        return


    def calib_ZeroStance_Axis0(self):
        des_angle = self.get_CurrentMotorAngle(0)

        self.set_ServoOn_Axis0()
        while True:
            cur_angle = self.get_CurrentMotorAngle(0)
            err_angle = des_angle - cur_angle

            threshold = 10.0 * DEG2RAD
            if threshold < abs(err_angle):
                self.ZeroStanceOffsetAngle_0 = -90.0 * DEG2RAD + cur_angle
                print("cur: {:.4}".format(cur_angle * RAD2DEG))
                # print("offset: {:.4}".format(self.ZeroStanceOffsetAngle_0 * RAD2DEG))
                break

            des_angle += 0.5 * DEG2RAD
            self.set_MotorAngle(0, des_angle)
            time.sleep(0.02)
            pass

        self.set_IDLE_Axis0()
        return


    def calib_ZeroStance_Axis1(self):
        des_angle = self.get_CurrentMotorAngle(1)

        self.set_ServoOn_Axis1()
        while True:
            cur_angle = self.get_CurrentMotorAngle(1)
            err_angle = des_angle - cur_angle

            threshold = 10.0 * DEG2RAD
            if threshold < abs(err_angle):
                self.ZeroStanceOffsetAngle_1 = 60.0 * DEG2RAD + cur_angle
                # print("offset: {:.4}".format(self.ZeroStanceOffsetAngle_0 * RAD2DEG))
                break

            des_angle += -0.5 * DEG2RAD
            self.set_MotorAngle(1, des_angle)
            time.sleep(0.02)
            pass

        self.set_IDLE_Axis1()
        return


    def set_JointAngle(self, joint_number: int = 0, des_angle: float = 0.0):
        if joint_number == 0:
            des_angle += self.ZeroStanceOffsetAngle_0
            self.set_MotorAngle(joint_number, des_angle)
        elif joint_number == 1:
            des_angle += self.ZeroStanceOffsetAngle_1
            self.set_MotorAngle(joint_number, des_angle)
        else:
            return 0.0
        return

    def set_JointAngle_Axis0(self, angle: float = 0.0):
        angle += self.ZeroStanceOffsetAngle_0
        self.set_MotorAngle(0, angle)
        return


    def set_JointAngle_Axis1(self, angle: float = 0.0):
        angle += self.ZeroStanceOffsetAngle_1
        self.set_MotorAngle(1, angle)
        return


    def set_JointAngles(self, des_angle_0, des_angle_1):
        self.set_JointAngle(0, des_angle_0)
        self.set_JointAngle(1, des_angle_1)
        return


    def get_CurrentJointAngle(self, motor_number=0):
        if motor_number == 0:
            cur_angle = self.Conv_pos2rad(self.my_drive.axis0.encoder.pos_estimate)
            cur_angle -= self.ZeroStanceOffsetAngle_0
            return cur_angle
        elif motor_number == 1:
            cur_angle = self.Conv_pos2rad(self.my_drive.axis1.encoder.pos_estimate)
            cur_angle -= self.ZeroStanceOffsetAngle_1
            return cur_angle

        return 0.0


    def MoveLinearPos(self, target_pos_x, target_pos_y, time_to_move=10.0):
        traj_x = TrajGen_HigherOrderPolynomials(
            self.EE_PosX, target_pos_x, time_to_move
        )
        traj_y = TrajGen_HigherOrderPolynomials(
            self.EE_PosY, target_pos_y, time_to_move
        )

        tk = TimeKeeper(0.01)
        start_time = time.time()
        while True:
            t = time.time() - start_time
            if time_to_move < t:
                break

            des_pos_x = traj_x.Get_Trajectory(t)
            des_pos_y = traj_y.Get_Trajectory(t)

            self.Set_Position(des_pos_x, des_pos_y)
            self.EE_PosX = des_pos_x
            self.EE_PosY = des_pos_y
            tk.SleepToKeep()
        return


    def moveLinearJoint(self, target_angle_0=0.0, target_angle_1=0.0, time_to_move=10.0):
        cur_angle_0 = self.get_CurrentJointAngle(0)
        cur_angle_1 = self.get_CurrentJointAngle(1)
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

            self.set_JointAngles(des_angle_0, des_angle_1)
            tk.SleepToKeep()
        return
    
    
    def moveHome(self):
        # Stand by
        self.moveLinearJoint(
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

