"""Unit conversions and data layouts of external libraries (definitions, not tunable settings)."""
import math

CM_PER_M = 100.0
MNM_PER_NM = 1000.0          # milli-newton-metre per newton-metre
MS_PER_S = 1000
DEG_PER_TURN = math.degrees(2.0 * math.pi)
UINT32_MASK = 0xFFFFFFFF     # MAVLink uint32 time fields

# MuJoCo free joint: qpos = [x y z | qw qx qy qz], qvel = [vx vy vz (world) | wx wy wz (body)]
MJ_POS, MJ_QUAT = slice(0, 3), slice(3, 7)
MJ_LIN_VEL, MJ_ANG_VEL = slice(0, 3), slice(3, 6)

# gym-pybullet-drones state vector (BaseAviary._getDroneStateVector)
PB_POS, PB_QUAT_XYZW = slice(0, 3), slice(3, 7)
PB_VEL, PB_ANG_VEL_WORLD = slice(10, 13), slice(13, 16)
PERCENT = 100                # colour channel 0..1 -> integer percent (material names)

# pybullet.getDebugVisualizerCamera() result layout
PB_CAM_YAW, PB_CAM_PITCH, PB_CAM_DIST, PB_CAM_TARGET = 8, 9, 10, 11

# MAVLink
US_PER_S = 1_000_000         # SET_MESSAGE_INTERVAL takes microseconds
MAV_FORCE_ARM_MAGIC = 21196  # MAV_CMD_COMPONENT_ARM_DISARM param2: arm/disarm without checks
GUID_OPTIONS_THRUST_BIT = 8  # ArduCopter GUID_OPTIONS bit 3: SET_ATTITUDE_TARGET thrust is thrust, not climb rate
COMMAND_LONG_PARAMS = 7     # COMMAND_LONG carries param1..param7
PB_CAM_PROJECTION, PB_CAM_FORWARD, PB_CAM_HORIZONTAL, PB_CAM_VERTICAL = 3, 5, 6, 7
PROJ_XX, PROJ_YY = 0, 5      # column-major 4x4 projection matrix: 1/(aspect*tan(fov/2)), 1/tan(fov/2)
