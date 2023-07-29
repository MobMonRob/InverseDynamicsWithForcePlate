from queue import Queue
import copy
from typing import TypeVar, Generic

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
