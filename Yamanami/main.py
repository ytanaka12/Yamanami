from odrive_driver import *
import myutil
import math


ODrive = ODriveDriver(odrive_serial_number="207835863056")

tk = myutil.TimeKeeper()
tk.SamplingTime = 0.005


def init():
    # global ODrive
    ODrive.connectToODrive()
    ODrive.clearError()

    ODrive.encoderOffsetCalibration_Axis0()
    ODrive.encoderOffsetCalibration_Axis1()

    ODrive.calib_ZeroStance_Axis0()
    ODrive.calib_ZeroStance_Axis1()

    ODrive.set_CLOSED_LOOP_CONTROL_BothAxis()
    return


def terminate():
    # global ODrive

    ODrive.set_MotorAngle(0, 0.0 * myutil.DEG2RAD)
    ODrive.set_MotorAngle(1, 0.0 * myutil.DEG2RAD)
    time.sleep(1)

    ODrive.set_IDLE_Axis0()
    ODrive.set_IDLE_Axis1()

    return


def moveHome():
    # Stand by
    ODrive.moveLinearM(
        target_angle_0=0.0 * myutil.DEG2RAD,
        target_angle_1=0.0 * myutil.DEG2RAD,
        time_to_move=1.0,
    )
    return


def main():
    print("hello world")

    init()

    tick = 0.0

    try:
        while True:
            angle_0 = 30.0 * math.sin(tick)
            angle_1 = 30.0 * math.cos(tick)
            ODrive.set_MotorAngle(0, angle_0 * myutil.DEG2RAD)
            ODrive.set_MotorAngle(1, angle_1 * myutil.DEG2RAD)
            tick += 0.05
            print("tick: {}".format(tick))

            tk.SleepToKeep()
            pass

    except KeyboardInterrupt:
        pass

    terminate()

    return


if __name__ == "__main__":
    main()
    pass
