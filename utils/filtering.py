# utils/filtering.py

from collections import deque


class MovingAverageFilter:
    def __init__(self, window_size: int):
        if not isinstance(window_size, int) or isinstance(window_size, bool) or window_size <= 0:
            raise ValueError("window_size must be a positive integer")
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self.sum = 0.0

    def filter(self, value: float) -> float:
        if len(self.buffer) == self.window_size:
            self.sum -= self.buffer[0]

        self.buffer.append(value)
        self.sum += value

        return self.sum / len(self.buffer)
