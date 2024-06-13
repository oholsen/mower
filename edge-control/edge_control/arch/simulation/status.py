from edge_control.models.turtle import Turtle
from edge_control.util.status import Status


class SimulationStatus:
    time = Status[float]()
    ok = Status[bool]()
    turtle = Status[Turtle]()
    # speeds, omega
    cut_power = Status[float]()
    
    @staticmethod
    def fault(t: float):
        SimulationStatus.time.check(t, "Simulated time")
        assert SimulationStatus.ok, "Status failed"
