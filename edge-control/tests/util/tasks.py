import asyncio
import logging

from edge_control.util.tasks import handle_exception, shutdown, start_task, start_tasks

logger = logging.getLogger(__name__)


async def task(delay: float, name: str):
    try:
        await asyncio.sleep(delay)
        raise Exception("FROM TASK " + name)
    except:
        # logger.exception("task() exception")
        raise


async def run1():
    # await asyncio.wait([task(2, "delay2"), task(20, "delay20")])

    t1 = start_task(task(2, "delay2"))
    t2 = start_task(task(20, "delay20"))
    await asyncio.sleep(60)
    # await asyncio.gather(t1, t2)


async def run2():
    done, pending = await asyncio.wait({task(2, "delay2"), task(20, "delay20")}, return_when=asyncio.FIRST_EXCEPTION)
    logger.debug("done = %s", done)
    for t in done:
        logger.debug("done = %s", t)
        # logger.debug("done = %r", t.exception())
        logger.debug("done = %s", t.result())
    logger.debug("pending = %s", pending)


async def run3():
    await start_tasks({task(2, "delay2"), task(8, "delay8")}, "run3")


def main():
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)
    asyncio.run(run3(), debug=True)
    # asyncio.run(shutdown(loop))


if __name__ == "__main__":
    main()
