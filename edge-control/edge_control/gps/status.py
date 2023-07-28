from edge_control.models.messages import SitePosition
from edge_control.util.status import Counter, Status

from .messages import GGA


class GpsStatus:
    gga = Status[GGA]()
    site = Status[SitePosition]()
    corrections = Counter()
    commands = Counter()
    responses = Counter()

    @staticmethod
    def fault(t: float):
        GpsStatus.gga.check(t, "GGA")
