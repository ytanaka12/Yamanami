from Yamanami.ODriveDriver import ODriveDriver
from Yamanami.myutil import *
import time


class RobotDriver(ODriveDriver):

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


    def moveHome(self):
        # Stand by
        self.moveLinearM(
            target_angle_0=0.0 * DEG2RAD,
            target_angle_1=0.0 * DEG2RAD,
            time_to_move=1.0,
        )
        return
    
    pass
