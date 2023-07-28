from abc import ABC, abstractmethod
from typing import Coroutine


class DriverStatus(ABC):
    @abstractmethod
    def as_dict(self) -> dict:
        raise NotImplementedError


class Driver(ABC):
    @abstractmethod
    def start(self) -> Coroutine:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> DriverStatus:
        raise NotImplementedError
