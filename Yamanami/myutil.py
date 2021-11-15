import math
import time


RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


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



