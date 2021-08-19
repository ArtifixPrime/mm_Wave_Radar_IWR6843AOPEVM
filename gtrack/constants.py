import numpy as np
import math

from numpy.core.fromnumeric import diagonal


# State vector constants
# docs/doxygen3D/html/group___g_t_r_a_c_k___a_l_g___e_x_t_e_r_n_a_l___d_a_t_a___s_t_r_u_c_t_u_r_e.html
# 2DV - Not supported
# 2DA - Supported
# 3DV - Not supported,
# 3DA - Supported
STATE_VECTORS_2DV = 0   # 2D motion model with constant velocity. State vector has four variables S={X,Y, Vx,Vy}
STATE_VECTORS_2DA = 1   # 2D motion model with constant acceleration. State vector has six variables S={X,Y, Vx,Vy, Ax,Ay}
STATE_VECTORS_3DV = 2   # 3D motion model with constant velocity. State vector has six variables S={X,Y,Z, Vx,Vy,Vz}
STATE_VECTORS_3DA = 3   # 3D motion model with constant acceleration. State vector has nine variables S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az}


# Verbose level constants
VERBOSE_NONE = 0        # none
VERBOSE_ERROR = 1       # only errors are reported
VERBOSE_WARNING = 2     # errors and warnings are reported
VERBOSE_DEBUG = 3       # errors, warnings, and state transitions are reported
VERBOSE_MATRIX = 4      # previous level plus all intermediate computation results are reported
VERBOSE_MAXIMUM = 5     # maximum amount of details are reported


# Track state constants
TRACK_STATE_FREE = 0
TRACK_STATE_INIT = 1
TRACK_STATE_DETECTION = 2
TRACK_STATE_ACTIVE = 3


# Velocity handling state
VELOCITY_INIT = 0
VELOCITY_RATE_FILTER = 1
VELOCITY_TRACKING = 2
VELOCITY_LOCKED = 3

# Maximum supported configurations
GTRACK_NUM_POINTS_MAX			  = 1000   # Maximum possible number of measurments point the algorithm will accept at configuration time
GTRACK_NUM_TRACKS_MAX			  = 200	   # Maximum possible number of tracking target the algorithm will accept at configuration time

# Target ID definitions
GTRACK_ID_GHOST_POINT_LIKELY      = 250    # Point is associated to the ghost right behind the target as single multipath reflection
GTRACK_ID_GHOST_POINT_BEHIND      = 251    # Point is associated to the ghost behind the target further away via multiple reflection paths
GTRACK_ID_RESERVED_GOOD_POINT     = 252    # Point definition is reserved
GTRACK_ID_POINT_TOO_WEAK          = 253    # Point is not associated, is too weak
GTRACK_ID_POINT_BEHIND_THE_WALL   = 254    # Point is not associated, is behind the wall
GTRACK_ID_POINT_NOT_ASSOCIATED    = 255    # Point is not associated, noise

# Benchmarking results (indexes for list which holds benchmarking results)
GTRACK_BENCHMARK_SETUP            = 0      # Cycle count at step setup
GTRACK_BENCHMARK_PREDICT          = 1      # Cycle count after predict function
GTRACK_BENCHMARK_ASSOCIATE        = 2      # Cycle count after associate function
GTRACK_BENCHMARK_ALLOCATE         = 3      # Cycle count after allocate function
GTRACK_BENCHMARK_UPDATE           = 4      # Cycle count after update function
GTRACK_BENCHMARK_PRESENCE         = 5      # Cycle count after presence detection function
GTRACK_BENCHMARK_REPORT           = 6      # Cycle count after report function


# Floats
DBL_DECIMAL_DIG = 17                       # num of decimal digits of rounding precision
DBL_DIG         = 15                       # num of decimal digits of precision
DBL_EPSILON     = 2.2204460492503131e-016  # smallest such that 1.0+DBL_EPSILON != 1.0
DBL_HAS_SUBNORM = 1                        # type does support subnormal numbers
DBL_MANT_DIG    = 53                       # num of bits in mantissa
DBL_MAX         = 1.7976931348623158e+308  # max value
DBL_MAX_10_EXP  = 308                      # max decimal exponent
DBL_MAX_EXP     = 1024                     # max binary exponent
DBL_MIN         = 2.2250738585072014e-308  # min positive value
DBL_MIN_10_EXP  = (-307)                   # min decimal exponent
DBL_MIN_EXP     = (-1021)                  # min binary exponent
_DBL_RADIX      = 2                        # exponent radix
DBL_TRUE_MIN    = 4.9406564584124654e-324  # min positive value

FLT_DECIMAL_DIG = 9                        # num of decimal digits of rounding precision
FLT_DIG         = 6                        # num of decimal digits of precision
FLT_EPSILON     = 1.192092896e-07          # smallest such that 1.0+FLT_EPSILON != 1.0
FLT_HAS_SUBNORM = 1                        # type does support subnormal numbers
FLT_GUARD       = 0
FLT_MANT_DIG    = 24                       # num of bits in mantissa
FLT_MAX         = 3.402823466e+38          # max value
FLT_MAX_10_EXP  = 38                       # max decimal exponent
FLT_MAX_EXP     = 128                      # max binary exponent
FLT_MIN         = 1.175494351e-38          # min normalized positive value
FLT_MIN_10_EXP  = (-37)                    # min decimal exponent
FLT_MIN_EXP     = (-125)                   # min binary exponent
FLT_NORMALIZE   = 0
FLT_RADIX       = 2                        # exponent radix
FLT_TRUE_MIN    = 1.401298464e-45          # min positive value

LDBL_DIG         = DBL_DIG                 # num of decimal digits of precision
LDBL_EPSILON     = DBL_EPSILON             # smallest such that 1.0+LDBL_EPSILON != 1.0
LDBL_HAS_SUBNORM = DBL_HAS_SUBNORM         # type does support subnormal numbers
LDBL_MANT_DIG    = DBL_MANT_DIG            # num of bits in mantissa
LDBL_MAX         = DBL_MAX                 # max value
LDBL_MAX_10_EXP  = DBL_MAX_10_EXP          # max decimal exponent
LDBL_MAX_EXP     = DBL_MAX_EXP             # max binary exponent
LDBL_MIN         = DBL_MIN                 # min normalized positive value
LDBL_MIN_10_EXP  = DBL_MIN_10_EXP          # min decimal exponent
LDBL_MIN_EXP     = DBL_MIN_EXP             # min binary exponent
_LDBL_RADIX      = _DBL_RADIX              # exponent radix
LDBL_TRUE_MIN    = DBL_TRUE_MIN            # min positive value

DECIMAL_DIG      = DBL_DECIMAL_DIG


# Ints
GTRACK_POS_DIMENSION = 0
GTRACK_VEL_DIMENSION = 1
GTRACK_ACC_DIMENSION = 2

GTRACK_SPREAD_ALPHA     	             =  0.05    # 5 percent
GTRACK_CONFIDENCE_ALPHA     	         =  0.1     # 10 percent
GTRACK_MIN_DISPERSION_ALPHA	             =  0.1     # 10 percent
GTRACK_MIN_POINTS_TO_UPDATE_DISPERSION   =  3       # 3 points

GTRACK_NOMINAL_ALLOCATION_RANGE     	 =  6.0     # Range for allocation SNR scaling

GTRACK_STATIC_MIN_CONIDENCE_LEVEL        =  0.8     # Minimal confidence level for static state



# Schemas
# Module instance shema
moduleInstanceSch = {
    'maxNumPoints': None,                   # Maximum number of measurement points per frame
    'maxNumTracks': None,                   # Maximum number of Tracking objects
    'params': {},                           # Tracking Unit Parameters
    'heartBeat': None,                      # TimeStamp
    'verbose': None,                        # Verboseness Mask
    'isCeilingMounted': None,               # Ceiling mount option
    'bestScore': [],                        # Array of best scores
    'bestIndex': [],                        # Array of best score authors (UIDs); UID -> Unit identifier
    'isUniqueIndex': [],                    # Bit Array of indicators whether best score author is unique
    'isStaticIndex': [],                    # Bit Array of indicators whether measurement point is associated with the static track
    'allocIndexCurrent': [],                # Temporary Array of measurement indices for current set-under-construction
    'allocIndexStored': [],                 # Temporary Array of measurement indices for stored set-under-construction
    'hTrack':  [],                          # List of all Tracking Unit instances
    'activeList': [],                       # List of currently active Tracking Units (UIDs)
    'freeList': [],                         # List of Free Tracking Units (UIDs)
    'uidElem': [],                          # Array of UID elements
    'targetDesc': [],                       # Array of Target descriptors
    'targetNumTotal': None,                 # Total number of tracked targets
    'targetNumCurrent': None,               # Number of currently tracked Targets
    'isPresenceDetectionEnabled': None,     # Presence detection enable
    'presenceDetectionRaw': None,           # Presence detection indication based on existing targets within the ocupance box
    'presenceDetectionInitial': None,       # The flag to indicate presence detection power up condition (no prior knowledge available)
    'presenceDetectionCounter': None,       # The counter for transaction presence ON => presence OFF
    'presenceDetectionOutput': None         # Presence detection output
}

# Unit instance schema
unitInstanceSch = {
    'uid': None,                            # Tracking Unit identifier
    'tid': None,                            # Target identifier
    'heartBeatCount': {},                   # TimeStamp
    'allocationTime': None,                 # Allocation time
    'allocationRange': None,                # Allocation range
    'allocationVelocity': None,             # Allocation radial velocity
    'estNumOfPoints': None,                 # Estimated number of points
    'state': None,                          # Current state
    'confidenceLevel': None,                # Target confidence level
    'confidenceLevelThreshold': None,       # Confidence Level threshold to determine sleep condition limit
    'isAssociationHeightIgnore': None,      # Indicates whether to ignore height dimension during 3D association or not (used in 3D ceiling mount configuration)
    'isAssociationGhostMarking': None,      # Indicates whether we mark ghosts behind the target (used in 2D case)
    'isEnablePointNumberEstimation': None,  # Indicates whether we estimate expected number of points
    'isTargetStatic': None,                 # Indicates whether target is currently static
    'skipPrediction': None,                 # Skip Prediction (when not enough reliable reflection points)
    'skipProcessNoiseEst': None,            # Skip Process Noise estimation (when not enough reliable reflection points)
    'isSnrWeighting': None,                 # Indicates whether SNR-based weighting or simple averaging is used to compute centroid center
    'stateVectorType': None,                # Requested State Vector type
    'currentStateVectorType': None,         # Current State Vector type
    'stateVectorDimNum': None,              # Number of Dimensions in State Vector, ex. 2 for constant velocity, 3 for constnat acceleration
    'stateVectorDimLength': None,           # Length of each dimensions in State Vector, ex. 2 for XY, 3 for XYZ
    'stateVectorLength': None,              # Length of State Vector
    'measurementVectorLength': None,        # Length of Measurement Vector
    'verbose': None,                        # Veboseness Mask
    'params': {},                           # Tracking Unit Parameters
    'velocityHandling': None,               # Current velocity handling State
    'initialRadialVelocity': None,          # Expected target radial velocity at the moment of detection, m/s
    'maxRadialVelocity': None,              # Absolute value of maximum radial velocity measure by the sensor, m/s
    'radialVelocityResolution': None,       # Radial velocity resolution of the sensor, m/s
    'rangeRate': None,                      # Current Range Rate value
    'minStaticVelocityOne': None,           # minimal velocity threshold to transition to static state (No points available)
    'minStaticVelocityTwo': None,           # minimal velocity threshold to transition to static state (Static points only)
    'estSpreadAlpha': None,                 # filter coefficient for spread estimation
    'numAssosiatedPoints': None,            # Number of assocuated dynamic points
    'detect2activeCount': None,             # Detection state count to active
    'detect2freeCount': None,               # Detection state count to free
    'active2freeCount': None,               # Active state count to free
    'sleep2freeCount': None,                # Active state static condition count to free
    'outside2freeCount': None,              # Outside boundary box count to free
    'maxAcceleration': [0.0, 0.0, 0.0],     # Configured target maximum acceleration
    'dt': None,                             # Configured Frame rate
    'F': None,                              # Pointer to current Transition matrix
    'Q': None,                              # Pointer to current Process Noise matrix
    'S_hat': None,                          # State matrix, estimated
    'S_apriori_hat': None,                  # State matrix, predicted
    'P_hat': None,                          # Process matrix, estimated
    'P_apriori_hat': None,                  # Process matrix, predicted
    'H_s': None,                            # Expected Measurement matrix
    'uCenter': [],                          # Calculated measurement centroid
    'uPos': [None, None, None],             # Calculated measurement centroid [posX, posY, posZ] in meters
    'H_limits': [],                         # Limits for associated measurments
    'estSpread': [],                        # Estimated spread of the measurements
    'estDim': None,                         # Estimated physical dimensions of the target
    'gD': None,                             # Group Dispersion matrix
    'gC': None,                             # Group Member Covariance matrix (between a member in measurment group and group centroid)
    'gC_inv': None,                         # Inverse of Group Covariance matrix
    'ec': None,                             # DEBUG, previous tick ec to report
    'gC_det': None,                         # determinant of Group Covariance matrix
    'G': None,                              # Gain used in association function
}

# Default values for configParams
# No presence detection
MAX_OCCUPANCY_BOXES = 2
boundaryBox = {
    'x1': 0.0, # left boundary in meters
    'x2': 0.0, # right boundary in meters
    'y1': 0.0, # near boundary in meters
    'y2': 0.0, # far boundary in meters
    'z1': 0.0, # bottom boundary in meters
    'z2': 0.0, # top boundary in meters
}

defaultOccupancyBoxes = []

for box in range(MAX_OCCUPANCY_BOXES):
    defaultOccupancyBoxes.append(boundaryBox)

defaultPresenceParams = {
    'pointsThre': 0,                        # occupancy threshold, number of points. Setting pointsThre to 0 disables presence detection
    'velocityThre': 0.0,                    # occupancy threshold, approaching velocity
    'on2offThre': 0,                        # occupancy on to off threshold
    'numOcupancyBoxes': 0,                  # Number of occulancy boxes. Presence detection algorithm will determine whether the combined shape is occupied. Setting numOccupancyBoxes to 0 disables presence detection.
    'boundaryBox': defaultOccupancyBoxes    # Scene occupancy boxes.
}

# No boundaries, no static boxes
MAX_BOUNDARY_BOXES = 2
MAX_STATIC_BOXES = 2
defaultBoundaryBoxes = []
defaultStaticBoxes = []

for box in range(MAX_BOUNDARY_BOXES):
    defaultBoundaryBoxes.append(boundaryBox)


for box in range(MAX_STATIC_BOXES):
    defaultStaticBoxes.append(boundaryBox)

defaultSceneryParams = {
    'sensorPosition': np.array([0.0, 0.0, 0.0]),
    'sensorOrientation': np.array([0.0, 0.0]),
    'numBoundaryBoxes': 0,
    'boundaryBox': defaultBoundaryBoxes,
    'numStaticBoxes': 0,
    'staticBox': defaultStaticBoxes
}

defaultGatingParams = {
    'gain': 2.0,
    'limits': {
        'depth': 3.0,
        'width': 2.0,
        'height': 2.0,
        'vel': 0.0
    }
}

defaultStateParams = {
    'det2actThre': 3,
    'det2freeThre': 3,
    'active2freeThre': 10,
    'static2freeThre': 40,
    'exit2freeThre': 5,
    'sleep2freeThre': 1000,
}

# At least 100 SNR, 100 SNR when obscured, 0.5 m/s, 5 points: up to 1m in distance, up to 2m/c in velocity
defaultAllocationParams = {
    'snrThre': 100.0,
    'snrThreObscured': 100.0,
    'velocityThre': 0.5,
    'pointsThre': 5,
    'maxDistanceThre': 1.0,
    'maxVelThre': 2.0
}

defaultUnrollingParams = {
    'alpha': 0.5,
    'confidence': 0.1
}

#zero3x3 = [
#    0.0, 0.0, 0.0,
#    0.0, 0.0, 0.0,
#    0.0, 0.0, 0.0]
zero3x3 = np.zeros((3,3), dtype=float)

#pinit6x6 = [
#    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
#    0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
#    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
#    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,]
pinitDiag = [0.0, 0.0, 0.5, 0.5, 1.0, 1.0]
pinit6x6 = np.diag(pinitDiag, dtype=float)


