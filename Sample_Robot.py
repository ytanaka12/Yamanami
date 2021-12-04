import math
import Yamanami
from Yamanami.myutil import *
import time

robot = Yamanami.RobotDriver(odrive_serial_number="207835863056")

tk = Yamanami.TimeKeeper()
tk.SamplingTime = 0.005


def main():
    robot.init()

    robot.setServoOn_Axis0()
    robot.setServoOn_Axis1()

    robot.moveHome()

    robot.moveJoint(0.0 * DEG2RAD, 30.0 * DEG2RAD, 2.0)

    tick = 0.0

    print("Press Ctrl + C to teminate.")
    try:
        while True:
            angle_0 = 30.0 * math.sin(tick)
            angle_1 = 30.0 * math.cos(tick)
            robot.setJointAngle_Axis0(angle_0 * DEG2RAD)
            robot.setJointAngle_Axis1(angle_1 * DEG2RAD)
            tick += 0.05

            tk.SleepToKeep()
            pass

    except KeyboardInterrupt:
        print("break")
        pass

    robot.terminate()

    return


if __name__ == "__main__":
    main()
    pass
