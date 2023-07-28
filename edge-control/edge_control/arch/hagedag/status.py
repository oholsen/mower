from edge_control.config import robot_config
from edge_control.util.status import Counter, Status

from . import messages


class HagedagStatus:
    reports = Counter()
    status = Status[messages.Status]()
    battery = Status[messages.Battery]()

    @staticmethod
    def fault(t: float):
        HagedagStatus.status.check(t, "Status")
        # TODO: add faults() to hagedag status giving "power fault" etc
        HagedagStatus.battery.check(t, "Battery")
        battery = HagedagStatus.battery.value
        assert battery is not None, "No battery status"
        assert battery.voltage > robot_config.battery_cutoff, "Battery low %.3fV" % battery.voltage
