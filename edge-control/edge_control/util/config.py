import logging
import logging.config
import os
from pathlib import Path
from typing import Type, TypeVar

from dacite import from_dict
from yaml import safe_load

logger = logging.getLogger(__name__)
T = TypeVar("T")


def filepath(filename: str) -> Path:
    """Use filename in CONFIG_DIR if it exists, otherwise revert to filename in current working directory"""
    directory = os.getenv("CONFIG_DIR")
    if directory:
        path = Path(directory).joinpath(filename)
        if path.exists():
            return path
    return Path(filename)


def read_config(filename: str, data_class: Type[T]) -> T:
    path = filepath(filename)
    logger.debug("Read config for %s from %s", data_class.__name__, path)
    with path.open() as file:
        data = safe_load(file)
        return from_dict(data_class=data_class, data=data)


def config_logging(config_file=None, verbose=False):
    try:
        if config_file:
            with open(config_file) as f:
                logging.config.dictConfig(safe_load(f))
                return
    except FileNotFoundError:
        pass
    logger_format = "%(asctime)s.%(msecs)03d %(levelname)-8s %(name)-22s %(message)s"
    logging.basicConfig(
        format=logger_format, datefmt="%Y-%m-%d %H:%M:%S", level=verbose and logging.DEBUG or logging.INFO
    )
