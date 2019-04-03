from enum import Enum
from geometry_msgs.msg import Pose, Point, Quaternion

# Shapes
class Shapes(Enum):
    unknown = -1
    triangle = 3
    square = 4
    pentagon = 5
    circle = 9


# Color Bounds
RED_UPPER = [20, 255, 255]
RED_LOWER = [317, 80, 80]
RED_UPPER_IMG = [10, 255, 255]
RED_LOWER_IMG = [335, 185, 50]
GREEN_UPPER_180 = [75, 255, 255]
GREEN_LOWER_180 = [48, 65, 101]
WHITE_UPPER = [255, 10, 255]
WHITE_LOWER = [0, 0, 185]

# Red Line Finder - Image Constants
TOP_CROP_FRAC = 0.0
BOTTOM_CROP_FRAC = 0.0
MASS_THRESHOLD = 100000
BOTTOM_DISTANCE_OFFSET = 0
HEIGHT = 0.5
BOTTOM_DISTANCE = -0.05
TOP_DISTANCE = 1.0

# Red Line Finder - Color Constants
H_MAX = 20
H_MIN = 317
S_MAX = 255
S_MIN = 180
V_MAX = 255
V_MIN = 80

# Location 2
g2_the_shape = Shapes.unknown

# Location 3
g3_found_flag = False
g3_turns = [69, 92, 92]

# Location 4
MARKER_POSE_TOPIC = "ar_pose_marker"
WAYPOINT_MAP = {
    "off_ramp": Pose(
        Point(-0.04193958488532269, -0.08882399277556528, 0.010199999999999999),
        Quaternion(0.0, 0.0, 0.34350948056213637, 0.9391492089992576),
    ),
    "1": Pose(
        Point(1.4128346917258936, 3.0314988225881017, 0.0102),
        Quaternion(0.0, 0.0, 0.11511062216559612, 0.9933526788933774),
    ),
    "2": Pose(
        Point(1.4974838033521558, 2.281408009296837, 0.0102),
        Quaternion(0.0, 0.0, 0.07185644361174354, 0.9974149846034358),
    ),
    "3": Pose(
        Point(1.6296398016279827, 1.4617992377591547, 0.0102),
        Quaternion(0.0, 0.0, 0.07214590937638982, 0.997394088492735),
    ),
    "4": Pose(
        Point(1.7891876967063711, 0.6209652252079119, 0.0102),
        Quaternion(0.0, 0.0, -0.005679318202515465, 0.9999838725423299),
    ),
    "5": Pose(
        Point(1.9150393764830873, -0.21134129264055662, 0.010199999999999999),
        #Quaternion(0.0, 0.0, -0.2439218105436791, 0.9697949011729714),
        Quaternion(0.0, 0.0, -0.005679318202515465, 0.9999838725423299),
    ),
    "6": Pose(
        Point(0.20461857551725132, 1.6533389602064918, 0.010200000000000004),
        Quaternion(0.0, 0.0, 0.9964777382330111, -0.08385772001445518),
    ),
    "7": Pose(
        Point(0.4272674336836944, 0.83219598399239, 0.0102),
        Quaternion(0.0, 0.0, 0.9947478059045937, -0.10235625358519646),
    ),
    "8": Pose(
        Point(1.0648070049458176, -0.302468245750806, 0.010199999999999999),
        Quaternion(0.0, 0.0, -0.6178099553656561, 0.7863274502718863),
    ),
    "far": Pose(
        Point(0.8348034063867399, 2.058496798133918, 0.010199999999999999),
        Quaternion(0.0, 0.0, 0.4270189450655432, 0.9042426779106981),
    ),
    "on_ramp": Pose(
        Point(-0.8334473332557941, 2.584811945485156, 0.0102),
        Quaternion(0.0, 0.0, -0.9617331318932912, 0.2739879249505739),
    ),
    "scan": Pose(
        Point(0.9380181464288736, 1.4855115054663668, 0.0102),
        Quaternion(0.0, 0.0, 0.0009838736033419004, 0.9999995159962491),
    ),
    "box_vantage": Pose(
        Point(1.9078119085847494, -0.25639876407921636, 0.0102),
        Quaternion(0.0, 0.0, 0.7627512753474568, 0.646691960639552),
    ),
    "scan_east": Pose(
        Point(1.117095360029464, 0.3325128316758337, 0.010200000000000002),
        Quaternion(0.0, 0.0, -0.5705387475488254, 0.8212706846986674),
    ),
    "scan_west": Pose(
        Point(0.6111472171378642, 2.0457316691198826, 0.0102),
        Quaternion(0.0, 0.0, 0.7258962437656054, 0.6878042187185134),
    ),
}

g4_box_id = -1
g4_box_left_side = None
g4_box_right_side = None
