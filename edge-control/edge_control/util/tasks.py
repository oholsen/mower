import asyncio
import logging
from typing import Coroutine

logger = logging.getLogger(__name__)


async def retry(fn, delay: float):
    while True:
        try:
            await fn()
        except (asyncio.CancelledError, KeyboardInterrupt):
            raise
        except:
            logger.exception("retry %s", fn)
        await asyncio.sleep(delay)


def start_tasks(aws, name: str = ""):
    if not name:
        name = str(aws)

    async def _wrap():
        done, pending = await asyncio.wait(aws, return_when=asyncio.FIRST_EXCEPTION)
        for task in done:
            logger.error("Task %s terminated: %s", task, task.result())
        for task in pending:
            logger.debug("Stopping task %s", task)
            task.cancel()
        await asyncio.wait(pending)
        logger.error("Task terminated: %s", name)

    logger.info("Task starting: %s", name)
    return asyncio.create_task(_wrap())


def start_task(coroutine: Coroutine, name: str = ""):
    # TODO: monitor tasks, catch exceptions from coroutine
    if not name:
        name = coroutine.__qualname__

    async def _wrap():
        try:
            await coroutine
        except KeyboardInterrupt:
            raise
        except asyncio.CancelledError:
            logger.info("Task cancelled: %s", name)
            pass
        except:
            logger.exception("Task error: %s", name)
        finally:
            logger.info("Task stopped: %s", name)

    logger.info("Task starting: %s", name)
    return asyncio.create_task(_wrap())


async def shutdown():
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    logger.info(f"Cancelling {len(tasks)} outstanding tasks")
    if tasks:
        for task in tasks:
            logger.debug("Cancel task %s: %s", task, task.result())
        for task in tasks:
            task.cancel()
        await asyncio.wait(tasks)


def handle_exception(loop, context):
    # context["message"] will always be there; but context["exception"] may not
    logger.error("Exception context: %s", context)
    msg = context.get("exception", context.get("message"))
    logger.error(f"Caught exception: {msg}")
    # logger.info("Shutting down...")
    # asyncio.create_task(shutdown(loop))
