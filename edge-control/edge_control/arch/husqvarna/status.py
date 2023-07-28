from edge_control.util.status import Status


class HusqvarnaStatus:
    speed = Status[float]()
    omega = Status[float]()

    @staticmethod
    def fault(t: float):
        pass
