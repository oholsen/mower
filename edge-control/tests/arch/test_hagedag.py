from edge_control.arch.hagedag import messages
from edge_control.arch.hagedag.status import HagedagStatus
from edge_control.robot import RobotState


def test_status():
    t = 20.0
    HagedagStatus.reports.inc(t)
    HagedagStatus.battery.set(messages.process("Battery 11.9"), t)
    HagedagStatus.status.set(messages.process("Status 0"), t)
    d = RobotState.as_dict(t)
    assert d["hagedag"] == {"battery": {"voltage": 11.9}, "reports": 1, "status": {"status": 0}}
