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
    ODriveSerialNumber: str = None


    def __init__(self, odrive_serial_number: str = "205F35863056"):
        self.ODriveSerialNumber = odrive_serial_number
        return

    def __del__(self):
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

        return


    ##
    ## Set Functions
    ##

    def set_SerialNumber(self, odrive_serial_number: str):
        self.ODriveSerialNumber = odrive_serial_number
        return

    def set_ServoOn_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_ServoOn_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_ServoOn_BothAxis(self):
        self.set_ServoOn_Axis0()
        self.set_ServoOn_Axis1()
        return


    def set_CLOSED_LOOP_CONTROL_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_CLOSED_LOOP_CONTROL_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return

    def set_CLOSED_LOOP_CONTROL_BothAxis(self):
        self.set_CLOSED_LOOP_CONTROL_Axis0()
        self.set_CLOSED_LOOP_CONTROL_Axis1()
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


    def set_MotorAngle(self, motor_number: int = 0, des_angle: float = 0.0) -> None:
        if motor_number == 0:
            self.my_drive.axis0.controller.input_pos = self.Conv_rad2pos(des_angle)
        elif motor_number == 1:
            self.my_drive.axis1.controller.input_pos = self.Conv_rad2pos(des_angle)
        else:
            sys.exit("Error: Invalid motor_number.")
        return


    def set_MotorAngles(self, des_angle_0, des_angle_1):
        self.set_MotorAngle(0, des_angle_0)
        self.set_MotorAngle(1, des_angle_1)
        return


    ##
    ## Get Functions
    ##

    def get_CurrentMotorAngle(self, motor_number: int = 0) -> float:
        if motor_number == 0:
            return self.Conv_pos2rad(self.my_drive.axis0.encoder.pos_estimate)
        elif motor_number == 1:
            return self.Conv_pos2rad(self.my_drive.axis1.encoder.pos_estimate)
        else:
            sys.exit("Error: Invalid motor_number.")
        return 0.0


    def get_CurrentMotorPos(self, motor_number=0) -> float:
        if motor_number == 0:
            return self.my_drive.axis0.encoder.pos_estimate
        elif motor_number == 1:
            return self.my_drive.axis1.encoder.pos_estimate
        else:
            sys.exit("Error: Invalid motor_number.")

        return 0.0


    def get_CurrentMotorAngles(self) -> (float, float):
        return self.get_CurrentMotorAngle(0), self.get_CurrentMotorAngle(1)

    def get_ODriveSerialNumber(self) -> str:
        return self.ODriveSerialNumber

    def get_ODriveInstance(self):
        return self.my_drive

    def Conv_pos2rad(self, motor_pos: float) -> float:
        return 2.0 * math.pi * motor_pos

    def Conv_rad2pos(self, angle: float) -> float:
        return angle / (2.0 * math.pi)

    def Distance(self, ax: float, ay: float, bx: float, by:float) -> float:
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

    #def MoveLinear(self, target_pos_x, target_pos_y, time_to_move=10.0):
    #    traj_x = TrajGen_HigherOrderPolynomials(
    #        self.EE_PosX, target_pos_x, time_to_move
    #    )
    #    traj_y = TrajGen_HigherOrderPolynomials(
    #        self.EE_PosY, target_pos_y, time_to_move
    #    )

    #    tk = TimeKeeper(0.01)
    #    start_time = time.time()
    #    while True:
    #        t = time.time() - start_time
    #        if time_to_move < t:
    #            break

    #        des_pos_x = traj_x.Get_Trajectory(t)
    #        des_pos_y = traj_y.Get_Trajectory(t)

    #        self.Set_Position(des_pos_x, des_pos_y)
    #        self.EE_PosX = des_pos_x
    #        self.EE_PosY = des_pos_y
    #        tk.SleepToKeep()
    #    return

    #def moveLinearM(self, target_angle_0=0.0, target_angle_1=0.0, time_to_move=10.0):
    #    cur_angle_0 = self.get_CurrentMotorAngle(0)
    #    cur_angle_1 = self.get_CurrentMotorAngle(1)
    #    traj_0 = TrajGen_HigherOrderPolynomials(
    #        cur_angle_0, target_angle_0, time_to_move
    #    )
    #    traj_1 = TrajGen_HigherOrderPolynomials(
    #        cur_angle_1, target_angle_1, time_to_move
    #    )

    #    tk = TimeKeeper(0.01)
    #    start_time = time.time()
    #    while True:
    #        t = time.time() - start_time
    #        if time_to_move < t:
    #            break

    #        des_angle_0 = traj_0.Get_Trajectory(t)
    #        des_angle_1 = traj_1.Get_Trajectory(t)

    #        self.set_MotorAngles(des_angle_0, des_angle_1)
    #        tk.SleepToKeep()
    #    return

    # Inverse Kinematics
    def inverseKinematics(self, x: float = 0.0, y: float = 0.0) -> (float, float):
        joint_angle_0 = 0.0
        joint_angle_1 = 0.0
        return joint_angle_0, joint_angle_1

    pass


#class TrajGen_HigherOrderPolynomials:
#    theta_0 = None
#    theta_f = None
#    theta_0_dot = 0.0
#    theta_f_dot = 0.0
#    theta_0_dotdot = 0.0
#    theta_f_dotdot = 0.0
#    t_f = None

#    a0 = None
#    a1 = None
#    a2 = None
#    a3 = None
#    a4 = None
#    a5 = None

#    def __init__(self, start_pos=0.0, end_pos=0.0, time_to_move=10.0):
#        self.Set_Conditions(start_pos, end_pos, time_to_move)
#        return

#    def Set_Conditions(self, start_pos=0.0, end_pos=0.0, time_to_move=10.0):
#        self.theta_0 = start_pos
#        self.theta_f = end_pos
#        if time_to_move < 0.1:
#            self.t_f = 0.1
#        else:
#            self.t_f = time_to_move

#        self.a0 = self.theta_0
#        self.a1 = self.theta_0_dot
#        self.a2 = self.theta_0_dotdot / 2.0
#        self.a3 = (
#            20.0 * self.theta_f
#            - 20.0 * self.theta_0
#            - (8.0 * self.theta_f_dot + 12.0 * self.theta_0_dot) * self.t_f
#            - (3.0 * self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)
#        ) / (2.0 * (self.t_f ** 3.0))
#        self.a4 = (
#            30.0 * self.theta_0
#            - 30.0 * self.theta_f
#            + (14.0 * self.theta_f_dot + 16.0 * self.theta_0_dot) * self.t_f
#            + (3.0 * self.theta_0_dotdot - 2.0 * self.theta_f_dotdot)
#            * (self.t_f ** 2.0)
#        ) / (2.0 * (self.t_f ** 4.0))
#        self.a5 = (
#            12.0 * self.theta_f
#            - 12.0 * self.theta_0
#            - (6.0 * self.theta_f_dot + 6.0 * self.theta_0_dot) * self.t_f
#            - (self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)
#        ) / (2.0 * (self.t_f ** 5.0))
#        return

#    def Get_Trajectory(self, time=0.0):
#        t = time
#        if self.t_f < t:
#            t = self.t_f

#        des_pos = self.a0
#        des_pos += self.a1 * t
#        des_pos += self.a2 * (t ** 2.0)
#        des_pos += self.a3 * (t ** 3.0)
#        des_pos += self.a4 * (t ** 4.0)
#        des_pos += self.a5 * (t ** 5.0)

#        return des_pos

#    def Get_t_f(self):
#        return self.t_f


# end class
