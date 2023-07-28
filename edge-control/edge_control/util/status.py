import time
from typing import Generic, Optional, Type, TypeVar

T = TypeVar("T")


def now():
    return time.time()


class Status(Generic[T]):
    # time is time.time()

    def __init__(self, ttl=60, value=None):
        # ttl is expiration time, after which it is an error condition if the status has not been updated.
        self.ttl = ttl
        self.value = value  # type: Optional[T]
        self.updated: Optional[float] = None
        self.changed: Optional[float] = None

    def clz(self) -> Type[T]:
        return self.__orig_class__.__args__[0]  # type: ignore

    def set(self, value: T, t: float = None):
        self.updated = t or now()
        if value != self.value:
            self.changed = self.updated
            self.value = value

    def present(self, t: float = None):
        return self.updated is not None and self.updated + self.ttl >= (t or now())

    def get(self, t: float = None) -> Optional[T]:
        return self.value if self.updated and (t or now()) <= self.updated + self.ttl else None

    def check(self, t: float, name: str):
        assert self.present(t), "Expired " + name
        assert self.value is not None, "Missing " + name

    def __str__(self):
        return f"Status({self.value}, {self.updated}, {self.ttl})"


class Counter(Status[int]):
    def __init__(self, ttl=60):
        super().__init__(ttl, 0)

    def inc(self, t: float = None) -> int:
        value = 1 if self.value is None else self.value + 1
        self.set(value, t)
        return value
