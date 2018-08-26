import time

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class Timer():
    __metaclass__ = Singleton

    def __init__(self):
        self.start_time = time.time()
        self.stop_time = 0

    def stopclock(self):
        self.stop_time = time.time()

    def get_elasped_time(self):
        if self.stop_time == 0:
            self.stopclock()
        return self.stop_time - self.start_time