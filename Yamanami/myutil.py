import math
import time

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


class Vector2D:
    x = 0.0
    y = 0.0
    pass


class Vector3D:
    x = 0.0
    y = 0.0
    z = 0.0
    pass


class TimeKeeper:
    SamplingTime = 0.01
    StartTime = None

    def __init__(self, sampling_time = 0.01):
        self.__SamplingTime = sampling_time
        self.StartTime = time.time()
        return

    def SleepToKeep(self):
        elapsed_time = time.time() - self.StartTime
        time_to_sleep = self.SamplingTime - elapsed_time
        if 0 < time_to_sleep:
            time.sleep(time_to_sleep)
        else:
            pass

        self.StartTime = time.time()
        return
    
    pass


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)


def distance2D(ax: float, ay: float, bx: float, by:float) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def distance3D(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)



