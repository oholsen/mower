import json
from typing import Any, List

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Envelope:
    type: str
    payload: Any
    timestamp: int

    def to_json(self):
        return json.dumps(asdict(self))

    def payload_to_json(self):
        return json.dumps(self.payload)


@dataclass(frozen=True)
class BatteryStatus:
    timestamp: int  # source time of battery status
    id: str
    chargePercent: float
    voltage: float


@dataclass(frozen=True)
class BatteriesStatus:
    batteries: List[BatteryStatus]


@dataclass(frozen=True)
class EdgeStatus:
    timestamp: int
    connected: bool
