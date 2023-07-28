import asyncio

import pytest

from edge_control.util import pubsub


@pytest.mark.asyncio
async def test_publish_subscribe():
    topic = pubsub.Topic[int]("topic")
    event = asyncio.Event()

    async def sub():
        stream = topic.stream()
        event.set()
        async for o in stream:
            return o

    task = asyncio.create_task(sub())
    # Create subscription before publishing
    await event.wait()
    await topic.publish(1)
    received = await task
    assert received == 1


@pytest.mark.asyncio
async def test_publish_invalid_type():
    topic = pubsub.Topic[int]("topic")
    with pytest.raises(AssertionError, match="Invalid type for topic"):
        await topic.publish("test")


@pytest.mark.asyncio
async def test_timeout():
    topic = pubsub.Topic[int]("topic")

    async def sub():
        async for o in topic.stream_timeout(timeout=0.1):
            return o

    task = asyncio.create_task(sub())
    received = await task
    assert received is None
