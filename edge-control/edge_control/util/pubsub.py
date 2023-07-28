import asyncio
import logging
from typing import AsyncGenerator, Generic, List, Optional, TypeVar

from aiostream.stream import merge as _merge

logger = logging.getLogger(__name__)
T = TypeVar("T")


class Topic(Generic[T]):
    def __init__(self, name: str):
        self.name = name
        self._queues: List[asyncio.Queue] = []

    async def publish(self, o: T):
        # logger.debug("PUBLISH %s %s %r", self.name, o, self._queues)
        # self.__orig_class__ not present in __init__(). mypy only checks generics with --strict!?
        clz = self.__orig_class__.__args__[0]  # type: ignore
        assert isinstance(o, clz), "Invalid type for topic %s: %s" % (self.name, o)
        if self._queues:
            await asyncio.wait([q.put(o) for q in self._queues])

    def subscription(self) -> asyncio.Queue:
        queue: asyncio.Queue = asyncio.Queue()
        self._queues.append(queue)
        return queue

    async def stream(self) -> AsyncGenerator[T, None]:
        queue = self.subscription()
        while True:
            yield await queue.get()

    async def stream_timeout(self, timeout: float) -> AsyncGenerator[Optional[T], None]:
        """yields value on every timeout"""
        queue = self.subscription()
        while True:
            try:
                yield await asyncio.wait_for(queue.get(), timeout=timeout)
            except asyncio.TimeoutError:
                yield None

    def __str__(self):
        return self.name


async def stream(queue: asyncio.Queue):
    while True:
        o = await queue.get()
        yield o


async def merge(*sources):
    combine = _merge(*sources)
    async with combine.stream() as streamer:
        async for o in streamer:
            yield o


async def dump(prefix: str, topic: Topic):
    async for s in topic.stream():
        logger.info("%s: %s", prefix, s)
