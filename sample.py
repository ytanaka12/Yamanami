#import myutil
import math
import Yamanami
from Yamanami.myutil import *
import time

robot = Yamanami.RobotDriver(odrive_serial_number="207835863056")

tk = Yamanami.TimeKeeper()
tk.SamplingTime = 0.005


def main():
    robot.init()

    tick = 0.0

    try:
        while True:
            angle_0 = 30.0 * math.sin(tick)
            angle_1 = 30.0 * math.cos(tick)
            robot.set_MotorAngle(0, angle_0 * DEG2RAD)
            robot.set_MotorAngle(1, angle_1 * DEG2RAD)
            tick += 0.05
            print("tick: {}".format(tick))

            tk.SleepToKeep()
            pass

    except KeyboardInterrupt:
        pass

    robot.terminate()

    return


if __name__ == "__main__":
    main()
    pass
