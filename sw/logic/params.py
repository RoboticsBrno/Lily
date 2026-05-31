from math import pi
import math

from geometry.shapes import Vector


# --- Physical ---

ROBOT_WHEEL_BASE: float = 0.249
ROBOT_TICKS_PER_METER: float = 496 / (0.0387 * math.pi)

ROBOT_BODY_RADIUS: float = 0.27 / 2


# --- Lidar ---

LIDAR_OFFSET: Vector = Vector(-0.05, 0)
LIDAR_SAMPLE_RATE: int = 4000

# --- Sim ---

SIM_LIDAR_MAX_RANGE: float = 3
SIM_LIDAR_ROTATION_HZ: float = 5
SIM_LIDAR_SAMPLE_RATE: int = LIDAR_SAMPLE_RATE
SIM_LIDAR_ANGLE_MIN: float = -pi
SIM_LIDAR_ANGLE_MAX: float = pi
SIM_LIDAR_DIST_NOISE: float = 0.001
SIM_LIDAR_ANGLE_NOISE: float = 0.001
SIM_LIDAR_RANDOM_DIST_PROB: float = 0.01
SIM_LIDAR_WORLD_MAX_INCIDENCE: float = pi / 6
SIM_LIDAR_BEAR_MAX_INCIDENCE: float = pi / 4

SIM_MOTOR_MAX_SPEED: float = 1

SIM_CLAW_OFFSET: Vector = Vector(0.08, 0.13)
SIM_CLAW_LENGTH: float = 0.15
SIM_CLAW_OPEN_ANGLE: float = 0.0
SIM_CLAW_CLOSED_ANGLE: float = 0.25 * math.pi

SIM_PUBLISH_HZ: float = 20.0
SIM_SIM_HZ: float = 1000.0


# --- Initial Pose ---

INITIAL_POSE_X: float = 0.2
INITIAL_POSE_Y: float = 0.2


# --- Default Bear ---

DEFAULT_BEAR_X: float = 0.05
DEFAULT_BEAR_Y: float = 1.45
DEFAULT_BEAR_RADIUS: float = 0.05


# --- Particle Filter ---

PF_NUM_PARTICLES: int = 500
PF_POSITION_NOISE: float = 0.005
PF_HEADING_NOISE: float = 0.01
PF_BLOCKED_HEADING_NOISE: float = 0.1
PF_SMOOTHING_ALPHA: float = 0.1
PF_LIDAR_SUBSAMPLING_FACTOR: int = LIDAR_SAMPLE_RATE // 40
PF_RESAMPLE_TINY_WEIGHT: float = 0.25
PF_RESAMPLE_TINY_THRESH: float = 0.5


# --- Bear Detector ---

BEAR_DBSCAN_EPS: float = 0.05
BEAR_DBSCAN_MIN_SAMPLES: int = 3
BEAR_MIN_DISTANCE: float = 0.1
BEAR_MAX_DISTANCE: float = 8
BEAR_FEATURE_DETECTION_POINTS: int = 7
BEAR_FEATURE_THRESHOLD: float = 0.015
BEAR_LINE_COVARIANCE_RATIO: float = 0.01
BEAR_MATCH_THRESHOLD: float = 0.3
BEAR_CONFIDENCE_THRESHOLD: float = 3.0
BEAR_MAX_CONFIDENCE: float = 5.0
BEAR_CONFIDENCE_INCREMENT: float = 1.0
BEAR_CONFIDENCE_DECREMENT: float = 1.0
BEAR_LERP_FACTOR: float = 0.5
BEAR_FEATURE_NORM_BIAS: float = 0.2


# --- Pure Pursuit ---

PP_LOOKAHEAD_DISTANCE: float = 0.18
PP_STEERING_GAIN: float = 4.0
PP_STEP_DISTANCE: float = 0.05
PP_BASE_POWER: float = 1.0


# --- Game State Machine ---

GAME_GOAL_TOLERANCE: float = 0.08
GAME_MAX_SPEED: float = 1
GAME_STARTUP_DELAY: float = 2.0
GAME_CLAW_CLOSE_DELAY: float = 2.0


# --- Keyboard control ---

KB_MOVE_POWER: float = 1.0
KB_TURN_POWER: float = 0.75
KB_MAX_SPEED: float = 1.2
