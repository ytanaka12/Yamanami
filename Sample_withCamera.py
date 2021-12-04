import math
import Yamanami
from Yamanami.myutil import *
import time
from multiprocessing import Process, Manager
import Sample_HandTracking as HandT
import SharedData
from DampedFilter import DampedFilter

robot = Yamanami.RobotDriver(odrive_serial_number="207835863056")

tk = Yamanami.TimeKeeper()
tk.SamplingTime = 0.005


CPARAM_P = 0.3
CPARAM_I = 0.001
CPARAM_D = 0.05
Ix = 0.0
Iy = 0.0
Bef_X = 0.5
Bef_Y = 0.5

def visual_servo(hand_pos_x: float = 0.5, hand_pos_y: float = 0.5):
    global Bef_X, Bef_Y
    ctrl_x = CPARAM_P * (0.5 - hand_pos_x) - CPARAM_D * (hand_pos_x - Bef_X)
    ctrl_y = CPARAM_P * (0.5 - hand_pos_y) - CPARAM_D * (hand_pos_y - Bef_Y)
    Bef_X = hand_pos_x
    Bef_Y = hand_pos_y
    global Ix, Iy
    Ix +=   CPARAM_I * (0.5 - hand_pos_x)
    Iy +=   CPARAM_I * (0.5 - hand_pos_y)

    cur_angle_0 = robot.getCurrentJointAngle_Axis0()
    cur_angle_1 = robot.getCurrentJointAngle_Axis1()
    robot.setJointAngle_Axis0(cur_angle_0 + ctrl_x + Ix)
    robot.setJointAngle_Axis1(cur_angle_1 - ctrl_y - Iy)
    return


def robot_main(sdict: dict = SharedData.SharedDict):
    robot.init()

    robot.setServoOn_Axis0()
    robot.setServoOn_Axis1()

    robot.moveHome()

    print("Press Ctrl + C to teminate.")
    try:
        while True:
            if sdict["terminate_flag"] == True:
                break

            if sdict["hand_isDetected"] == True and sdict["tracking_flag"] == True:
                visual_servo(sdict["hand_pos_x"], sdict["hand_pos_y"])
                pass

            tk.SleepToKeep()
            pass

    except KeyboardInterrupt:
        print("break")
        pass

    robot.terminate()

    return


def main():
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



