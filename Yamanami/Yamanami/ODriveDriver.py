import sys
import odrive
from odrive.enums import *
import time
import math
from Yamanami.myutil import *

##
# @class ODriveDriver
# @brief This class wraps function of odrive.
# @details
class ODriveDriver:
    my_drive = None  # ODrive Instance
    ODriveSerialNumber = None

    # for unintentional one rotation
    MotorAngleOffset_0 = 0.0
    MotorAngleOffset_1 = 0.0

    # for zero stance of rinta
    # You must adjust below two offset angle by your hands before the system runs.
    ZeroStanceOffsetAngle_0 = 0.0 * DEG2RAD  # rad
    ZeroStanceOffsetAngle_1 = 0.0 * DEG2RAD

    NOZZLE_LENGTH = 0.08  # (m)

    def __init__(self, odrive_serial_number="205F35863056"):
        self.ODriveSerialNumber = odrive_serial_number
        # self.ConnectToODrive()
        return

    def __del__(self):
        #self.set_IDLE_BothAxis()
        return

    def connectToODrive(self):
        print(
            "Connecting an odrive, Serial Number: {0} ...".format(
                self.ODriveSerialNumber
            )
        )
        self.my_drive = odrive.find_any(serial_number=self.ODriveSerialNumber)
        print("Connected to Serial Number: {0}.".format(self.ODriveSerialNumber))
        return

    def clearError(self):
        self.my_drive.axis0.error = 0
        return

    def encoderOffsetCalibration_BothAxis(self):
        # Calibrate motor and wait for it to finish
        if (
            self.my_drive.axis0.encoder.is_ready == False
            or self.my_drive.axis1.encoder.is_ready == False
        ):
            print("Start Encoder Calibration...")
            self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            self.my_drive.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if (
                self.my_drive.axis0.encoder.is_ready == False
                or self.my_drive.axis1.encoder.is_ready == False
            ):
                print("Calibration fail\n")
                return -1
            else:
                print("Encoders have been Calibrated !!\n")
        else:
            print("Already Calibrated !!\n")
        return

    def encoderOffsetCalibration_Axis0(self):
        if self.my_drive.axis0.encoder.is_ready == False:
            print("Start Encoder Offset Calibration on Axis 0...")
            self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if self.my_drive.axis0.encoder.is_ready == False:
                print("Encoder Offset Calibration on Axis 0 was failed")
                return -1
            else:
                print("Encoder Offset Calibration on Axis 0 was Success.")
        else:
            print("Encoder offset was already calibrated.")
            pass

        # m_angle = self.get_CurrentMotorAngle(motor_number = 0)
        # if 180.0 < m_angle * RAD2DEG:
        #    self.MotorAngleOffset_0 = 360.0 * DEG2RAD

        return

    def encoderOffsetCalibration_Axis1(self):
        if self.my_drive.axis1.encoder.is_ready == False:
            print("Start Encoder Offset Calibration on Axis 1...")
            self.my_drive.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.my_drive.axis1.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if self.my_drive.axis1.encoder.is_ready == False:
                print("Encoder Offset Calibration on Axis 1 was failed")
                return -1
            else:
                print("Encoder Offset Calibration on Axis 1 was Success.")
        else:
            print("Encoder offset was already calibrated.")
            pass

        # m_angle = self.get_CurrentMotorAngle(motor_number = 1)
        # if 180.0 < m_angle * RAD2DEG:
        #    self.MotorAngleOffset_1 = 360.0 * DEG2RAD

        return

    def calib_ZeroStance_Axis0(self):
        des_angle = self.get_CurrentMotorAngle(0)

        self.set_ServoOn_Axis0()
        while True:
            cur_angle = self.get_CurrentMotorAngle(0)
            err_angle = des_angle - cur_angle
            # print("cur: {:.4}".format(cur_angle * RAD2DEG))
            # print("err: {:.4}".format(err_angle * RAD2DEG))

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
            # print("cur: {:.4}".format(cur_angle * RAD2DEG))
            # print("err: {:.4}".format(err_angle * RAD2DEG))

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

    ##
    ## Set Functions
    ##

    def set_SerialNumber(self, odrive_serial_number):
        self.ODriveSerialNumber = odrive_serial_number
        return

    def set_ServoOn_BothAxis(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_ServoOn_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_ServoOn_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_CLOSED_LOOP_CONTROL_BothAxis(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_CLOSED_LOOP_CONTROL_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_CLOSED_LOOP_CONTROL_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_IDLE_BothAxis(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_IDLE
        self.my_drive.axis1.requested_state = AXIS_STATE_IDLE
        return

    def set_IDLE_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_IDLE
        return

    def set_IDLE_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_IDLE
        return

    # des_angle [rad]
    def set_MotorAngle(self, motor_number=0, des_angle=0.0):
        if motor_number == 0:
            des_angle += self.MotorAngleOffset_0
            des_angle += self.ZeroStanceOffsetAngle_0
            self.my_drive.axis0.controller.input_pos = self.Conv_rad2pos(des_angle)
        elif motor_number == 1:
            des_angle += self.MotorAngleOffset_1
            des_angle += self.ZeroStanceOffsetAngle_1
            self.my_drive.axis1.controller.input_pos = self.Conv_rad2pos(des_angle)
        else:
            return 0.0
        return

    # des_angle_0 [rad]
    # des_angle_1 [rad]
    def set_MotorAngles(self, des_angle_0, des_angle_1):
        self.set_MotorAngle(0, des_angle_0)
        self.set_MotorAngle(1, des_angle_1)
        return

    def set_Position(self, des_pos_x, des_pos_y):
        return

    ##
    ## Get Functions
    ##

    def get_CurrentMotorAngle(self, motor_number=0):
        if motor_number == 0:
            cur_angle = self.Conv_pos2rad(self.my_drive.axis0.encoder.pos_estimate)
            cur_angle -= self.MotorAngleOffset_0
            cur_angle -= self.ZeroStanceOffsetAngle_0
            return cur_angle
        elif motor_number == 1:
            cur_angle = self.Conv_pos2rad(self.my_drive.axis1.encoder.pos_estimate)
            cur_angle -= self.MotorAngleOffset_1
            cur_angle -= self.ZeroStanceOffsetAngle_1
            return cur_angle

        return 0.0

    def get_CurrentMotorPos(self, motor_number=0):
        if motor_number == 0:
            return self.my_drive.axis0.encoder.pos_estimate
        elif motor_number == 1:
            return self.my_drive.axis1.encoder.pos_estimate

        return 0.0

    def get_CurrentMotorAngles(self):
        return self.get_CurrentMotorAngle(0), self.get_CurrentMotorAngle(1)

    def get_ODriveSerialNumber(self):
        return self.ODriveSerialNumber

    def get_ODriveInstance(self):
        return self.my_drive

    def Conv_pos2rad(self, motor_pos):
        return 2.0 * math.pi * motor_pos

    def Conv_rad2pos(self, angle):
        return angle / (2.0 * math.pi)

    def Distance(self, ax, ay, bx, by):
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

    def MoveLinear(self, target_pos_x, target_pos_y, time_to_move=10.0):
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

    def moveLinearM(self, target_angle_0=0.0, target_angle_1=0.0, time_to_move=10.0):
        cur_angle_0 = self.get_CurrentMotorAngle(0)
        cur_angle_1 = self.get_CurrentMotorAngle(1)
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

            self.set_MotorAngles(des_angle_0, des_angle_1)
            tk.SleepToKeep()
        return

    # Inverse Kinematics of rinta
    def inverseKinematics(self, x=0.0, y=0.0):
        buf = x / self.NOZZLE_LENGTH
        angle_1 = math.asin(buf)
        buf = y / (self.NOZZLE_LENGTH * math.cos(angle_1))
        angle_0 = math.asin(buf)
        return angle_0, angle_1

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


# end class
