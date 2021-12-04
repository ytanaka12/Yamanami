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
        print("Connecting an odrive, Serial Number: {0} ...".format(self.ODriveSerialNumber))
        self.my_drive = odrive.find_any(serial_number=self.ODriveSerialNumber)
        print("Connected to Serial Number: {0}.".format(self.ODriveSerialNumber))
        self.setODriveParameters()
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

    def setSerialNumber(self, odrive_serial_number: str):
        self.ODriveSerialNumber = odrive_serial_number
        return


    def setServoOn_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return


    def setServoOn_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return


    def setServoOff_Axis0(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_IDLE
        return


    def setServoOff_Axis1(self):
        self.my_drive.axis1.requested_state = AXIS_STATE_IDLE
        return


    def setMotorAngle_Axis0(self, angle: float = 0.0):
        self.my_drive.axis0.controller.input_pos = self.Conv_rad2pos(angle)
        return



    def setMotorAngle_Axis1(self, angle: float = 0.0):
        self.my_drive.axis1.controller.input_pos = self.Conv_rad2pos(angle)
        return



    ##
    ## Get Functions
    ##

    def getCurrentMotorAngle_Axis0(self):
        return self.Conv_pos2rad(self.getCurrentMotorPos_Axis0())


    def getCurrentMotorAngle_Axis1(self):
        return self.Conv_pos2rad(self.getCurrentMotorPos_Axis1())


    def getCurrentMotorPos_Axis0(self):
        return self.my_drive.axis0.encoder.pos_estimate


    def getCurrentMotorPos_Axis1(self):
        return self.my_drive.axis1.encoder.pos_estimate


    def get_ODriveSerialNumber(self) -> str:
        return self.ODriveSerialNumber

    def get_ODriveInstance(self):
        return self.my_drive

    def Conv_pos2rad(self, motor_pos: float) -> float:
        return 2.0 * math.pi * motor_pos

    def Conv_rad2pos(self, angle: float) -> float:
        return angle / (2.0 * math.pi)


    def setODriveParameters(self):
        self.my_drive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.my_drive.axis0.motor.config.requested_current_range = 9
        self.my_drive.axis0.motor.config.current_lim = 20
        self.my_drive.axis0.motor.config.resistance_calib_max_voltage = 8
        self.my_drive.axis0.motor.config.calibration_current = 10
        self.my_drive.axis0.motor.config.pole_pairs = 14/2
        self.my_drive.axis0.motor.config.current_control_bandwidth = 300.0
        self.my_drive.axis0.motor.config.pre_calibrated = True
        self.my_drive.axis0.encoder.config.cpr = 2**12
        self.my_drive.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        self.my_drive.axis0.encoder.config.calib_range = 0.8
        self.my_drive.axis0.encoder.config.calib_scan_distance = 5.0
        self.my_drive.axis0.encoder.config.calib_scan_omega = 12.0
        self.my_drive.axis0.controller.config.pos_gain = 120
        self.my_drive.axis0.controller.config.vel_gain = 0.1
        self.my_drive.axis0.controller.config.vel_integrator_gain = 0.0
        self.my_drive.axis0.controller.config.vel_limit = 1000.0
        self.my_drive.axis0.controller.config.input_filter_bandwidth=100000

        self.my_drive.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.my_drive.axis1.motor.config.requested_current_range = 9
        self.my_drive.axis1.motor.config.current_lim = 20
        self.my_drive.axis1.motor.config.resistance_calib_max_voltage = 8
        self.my_drive.axis1.motor.config.calibration_current = 10
        self.my_drive.axis1.motor.config.pole_pairs = 14/2
        self.my_drive.axis1.motor.config.current_control_bandwidth = 300.0
        self.my_drive.axis1.motor.config.pre_calibrated = True
        self.my_drive.axis1.encoder.config.cpr = 2**12
        self.my_drive.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        self.my_drive.axis1.encoder.config.calib_range = 0.8
        self.my_drive.axis1.encoder.config.calib_scan_distance = 5.0
        self.my_drive.axis1.encoder.config.calib_scan_omega = 12.0
        self.my_drive.axis1.controller.config.pos_gain = 120
        self.my_drive.axis1.controller.config.vel_gain = 0.1
        self.my_drive.axis1.controller.config.vel_integrator_gain = 0.0
        self.my_drive.axis1.controller.config.vel_limit = 1000.0
        self.my_drive.axis1.controller.config.input_filter_bandwidth=100000

        self.my_drive.axis0.controller.config.control_mode = INPUT_MODE_POS_FILTER
        self.my_drive.axis1.controller.config.control_mode = INPUT_MODE_POS_FILTER

        self.my_drive.save_configuration()

        print("ODrive parameters setup is done.")
        return


    pass #end of class: ODriveDriver

