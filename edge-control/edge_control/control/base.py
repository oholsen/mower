import logging
from abc import ABC
from typing import Generator, Optional, Tuple

from edge_control.models.messages import ToRobot
from edge_control.models.state import State

log = logging.getLogger(__name__)


class ControlException(Exception):
    pass


class Control(ABC):
    def update(self, t: float, state: State) -> Optional[ToRobot]:
        raise NotImplemented

    def end(self, _t: float, _state: State) -> bool:
        return False

    def __str__(self):
        return self.__class__.__name__


class StateIndependentControl(Control):
    """Does not need a position/state to update() or end()"""

    pass


class CompositeControl(Control):
    """Use sequence of controls, end() on the current control triggers using next control"""

    # TODO: track which control is executed, including nested composite controls
    # Can also name a composite... controls may be a function, enough???

    def __init__(self, controls: Generator[Control, Tuple[float, State], None]):
        self.controls = controls
        self.control = next(self.controls)
        self.stopped = False
        log.info("Starting %s", self.control)

    def __str__(self):
        return f"CompositeControl({self.controls},{self.control})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        assert self.control is not None
        # log.debug("Composite update %s %f %s", self.control, t, state)
        while self.control.end(t, state):
            # log.debug("Composite end %s %f %s", self.control, t, state)
            try:
                self.control = self.controls.send((t, state))
            except (StopAsyncIteration, StopIteration):
                log.info("Stopped CompositeControl")
                self.stopped = True
                return None
            log.info("Starting %s %s", t, self.control)
        return self.control.update(t, state)

    def end(self, t: float, state) -> bool:
        return self.stopped
