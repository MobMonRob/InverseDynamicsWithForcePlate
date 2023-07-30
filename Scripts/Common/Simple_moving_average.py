from queue import Queue
import copy
from typing import TypeVar, Generic
from Common.Ros_msg_types.vicon_data_publisher.msg._Force_plate_data import Force_plate_data

class SimpleMovingAverage():
    def __init__(self, window_size: int):
        self.window_size: int = window_size
        self.queue: Queue = Queue(window_size)
        self.average: float = 0.0
        return


    def process(self, new: float) -> float:
        # init averaging window
        if (self.queue.qsize() < self.window_size):
            self.queue.put(new)
            self.average = self.average + (new / self.window_size)
            return self.average

        # move averaging window
        old = self.queue.get()
        self.queue.put(new)

        # apply averaging
        self.average = self.average + (new - old) / self.window_size
        
        return self.average


T = TypeVar('T')
class SimpleMovingAverageOnObjects(Generic[T]):
    def __init__(self, window_size: int, sample: T):
        self.smas: "dict[T, SimpleMovingAverage]" = dict()
        for fieldName in sample.__slots__:
            if isinstance(getattr(sample, fieldName), float):
                self.smas[fieldName] = SimpleMovingAverage(window_size)
    

    def process(self, new: T) -> T:
        average: T = copy.copy(new)

        for fieldName, sma in self.smas.items():
            fieldValue = getattr(new, fieldName)
            fieldAverage: float = sma.process(fieldValue)
            setattr(average, fieldName, fieldAverage)

        return average


def calculate_sma(frameNumbers_to_forcePlateData_mean: "dict[int, Force_plate_data]") -> "dict[int, Force_plate_data]":
    sma: SimpleMovingAverageOnObjects[Force_plate_data] = SimpleMovingAverageOnObjects[Force_plate_data](1000, Force_plate_data())
    
    frameNumbers_to_forcePlateData_sma: dict[int, Force_plate_data] = dict()
    for frameNumber, forcePlateData in frameNumbers_to_forcePlateData_mean.items():
        forcePlataData_sma: Force_plate_data = sma.process(forcePlateData)
        forcePlataData_sma.frameNumber = frameNumber
        frameNumbers_to_forcePlateData_sma[frameNumber] = forcePlataData_sma

    return sma

