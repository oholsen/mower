from .gps.messages import GGA
from .models import cdf
from .models.messages import FromRobot, LightsCommand, MissionCommand, Odometry, SitePosition, ToRobot
from .models.ptz import ToPanTiltZoom
from .models.state import State
from .util.inrobot import BatteriesStatus, EdgeStatus, Envelope
from .util.pubsub import Topic

# robot commands - generic for several robots
robot_command: Topic[ToRobot] = Topic[ToRobot]("robot_command")
mission_command: Topic[MissionCommand] = Topic[MissionCommand]("mission_command")

# robot state - generic for several robots
inrobot_api: Topic[Envelope] = Topic[Envelope]("inrobot_api")
robot_state: Topic[FromRobot] = Topic[FromRobot]("robot_state")
edge_status: Topic[EdgeStatus] = Topic[EdgeStatus]("edge_status")
batteries_status: Topic[BatteriesStatus] = Topic[BatteriesStatus]("batteries_status")

# tracking position, speed and heading - specific for robot type (e.g. two-wheeled turtle)
odometry: Topic[Odometry] = Topic[Odometry]("odometry")
robot_tracking: Topic[State] = Topic[State]("robot_tracking")

# site and world positions
site_position: Topic[SitePosition] = Topic[SitePosition]("site_position")
gps_position: Topic[GGA] = Topic[GGA]("gps_position")
gps_command = Topic[bytes]("gps_command")  # RTK corrections
gps_nmea = Topic[str]("gps_nmea")  # NMEA sentences

# payloads
lights: Topic[LightsCommand] = Topic[LightsCommand]("lights")
ptz_command = Topic[ToPanTiltZoom]("ptz_command")

# mission reports to CDF via extractor
missions = Topic[cdf.Mission]("missions")
gauge_captures = Topic[cdf.GaugeCapture]("gauge_captures")
