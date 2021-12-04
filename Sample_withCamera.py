import math
import Yamanami
from Yamanami.myutil import *
import time
from multiprocessing import Process, Manager
import Sample_HandTracking as HandT
import SharedData

robot = Yamanami.RobotDriver(odrive_serial_number="207835863056")

tk = Yamanami.TimeKeeper()
tk.SamplingTime = 0.005


def visual_servo(target_x: float = 0.5, target_y: float = 0.5):

    return


def robot_main(sdict: dict = SharedData.SharedDict):
    robot.init()

    robot.setServoOn_Axis0()
    robot.setServoOn_Axis1()

    robot.moveHome()

    robot.moveJoint(0.0 * DEG2RAD, 30.0 * DEG2RAD, 2.0)

    tick = 0.0

    print("Press Ctrl + C to teminate.")
    try:
        while True:
            if sdict["terminate_flag"] == True:
                break

            if sdict["hand_isDetected"] == True:
                print("Tracking Flag: {}".format(sdict["tracking_flag"]))
                #print("hand pos: {:.3f} / {:.3f}".format(sdict["hand_pos_x"], sdict["hand_pos_y"]))
                pass

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


def main():
    buf_dict = dict(pos_x=0.01, pos_y=0.0, isRock=False)
    sdict = Manager().dict(SharedData.SharedDict)

    process_robot = Process(target = robot_main, args = [sdict] )
    process_handtracking = Process(target = HandT.main, args = [sdict] )

    process_robot.start()
    process_handtracking.start()

    try:
        while True:
            if(process_robot.is_alive() == False
               and process_handtracking.is_alive() == False
               ):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Break Process")

    process_robot.join()
    process_handtracking.join()
    process_robot.terminate()
    process_handtracking.terminate()

    print("Processes have been terminated.")
    return


if __name__ == "__main__":
    main()
    pass



