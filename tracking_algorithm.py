# imports
import math
import numpy as np


# Scate vector constants
# docs/doxygen3D/html/group___g_t_r_a_c_k___a_l_g___e_x_t_e_r_n_a_l___d_a_t_a___s_t_r_u_c_t_u_r_e.html
# 2DV - Not supported		
# 2DA - Supported
# 3DV - Not supported,
# 3DA - Supported
STATE_VECTORS_2DV = 0;  # 2D motion model with constant velocity. State vector has four variables S={X,Y, Vx,Vy}
STATE_VECTORS_2DA = 1;  # 2D motion model with constant acceleration. State vector has six variables S={X,Y, Vx,Vy, Ax,Ay}
STATE_VECTORS_3DV = 2;  # 3D motion model with constant velocity. State vector has six variables S={X,Y,Z, Vx,Vy,Vz}
STATE_VECTORS_3DA = 3;  # 3D motion model with constant acceleration. State vector has nine variables S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az}


# Verbose level constants
VERBOSE_NONE = 0;       # none
VERBOSE_ERROR = 1;      # only errors are reported
VERBOSE_WARNING = 2;    # errors and warnings are reported
VERBOSE_DEBUG = 3;      # errors, warnings, and state transitions are reported
VERBOSE_MATRIX = 4;     # previous level plus all intermediate computation results are reported
VERBOSE_MAXIMUM = 5;    # maximum amount of details are reported



# Default values for configParams
# No presence detection
MAX_OCCUPANCY_BOXES = 2;
boundaryBox = {
    'x1': 0.0, # left boundary in meters
    'x2': 0.0, # right boundary in meters
    'y1': 0.0, # near boundary in meters
    'y2': 0.0, # far boundary in meters
    'z1': 0.0, # bottom boundary in meters
    'z2': 0.0, # top boundary in meters
}

defaultOccupancyBoxes = []

for box in MAX_OCCUPANCY_BOXES:
    defaultOccupancyBoxes[box] = boundaryBox

defaultPresenceParams = {
    'pointsThre': 0,                        # occupancy threshold, number of points. Setting pointsThre to 0 disables presence detection
    'velocityThre': 0.0,                    # occupancy threshold, approaching velocity
    'on2offThre': 0,                        # occupancy on to off threshold
    'numOcupancyBoxes': 0,                  # Number of occulancy boxes. Presence detection algorithm will determine whether the combined shape is occupied. Setting numOccupancyBoxes to 0 disables presence detection.
    'boundaryBox': defaultOccupancyBoxes    # Scene occupancy boxes.
}

# No boundaries, no static boxes
MAX_BOUNDARY_BOXES = 2;
MAX_STATIC_BOXES = 2;
defaultBoundaryBoxes = []
defaultStaticBoxes = []

for box in MAX_BOUNDARY_BOXES:
    defaultBoundaryBoxes[box] = boundaryBox

for box in MAX_STATIC_BOXES:
    defaultStaticBoxes[box] = boundaryBox

defaultSceneryParams = {
    'sensorPosition': [0.0, 0.0, 0.0],
    'sensorOrientation': [0.0, 0.0],
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

zero3x3 = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
]

pinit6x6 = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
]



def gtrack_create(moduleConfig):
    """Creates a module instance with parameter values that are constant for each of the tracked units within the module.
    These parameters are:
        a) Default parameters for gating, statem velocity unrolling, allocation and presence.
        b) Parameters for wall-mount or ceiling-mount configuration of the sensor (based on height and elevation tilt).
        c) Initialize Process Noise Matrix Q and State transition Matrix F.

    Args:
        configParams (dict): Contains the parameters that are exposed by gtrack algorithm.
                             The configuration does not need to persist.
                             Advanced configuration structure can be set to NULL to use the default one.
                             Any field within Advanced configuration can also be set to NULL to use the default values for the field.
    """

    # Module instance initialization
    # docs/doxygen3D/html/struct_gtrack_module_instance.html
    moduleInstance = {
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
        'allicIndexStored': [],                 # Temporary Array of measurement indices for stored set-under-construction
        'hTrack':  [],                          # List of all Tracking Units
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

    # Exceptions
    #if(moduleConfig['maxNumPoints'] > NUM_POINTS_MAX):
        # Invalid argument or the configured number of measurement points exceeds the maximum allowed number of measurement points.
        # exit and delete module instance
        # pass

    #if(moduleConfig['maxNumTracks'] > NUM_TRACKS_MAX):
        # Invalid argument or the configured number of measurement points exceeds the maximum allowed number of measurement points.
        # exit and delete module instance
        # pass

    # Set instance parameters from config
    moduleInstance['maxNumPoints'] = moduleConfig['maxNumPoints']
    moduleInstance['maxNumTracks'] = moduleConfig['maxNumTracks']

    moduleInstance['heartBeat'] = 0

    # Set instance parameters from default parameters
    moduleInstance['params']['gatingParams'] = defaultGatingParams
    moduleInstance['params']['stateParams'] = defaultStateParams
    moduleInstance['params']['unrollingParams'] = defaultUnrollingParams
    moduleInstance['params']['allocationParams'] = defaultAllocationParams
    moduleInstance['params']['sceneryParams'] = defaultSceneryParams
    moduleInstance['params']['presenceParams'] = defaultPresenceParams


    # Overwrite default parameters config parameters if they exist
    if(moduleConfig['advParams']):
        if(moduleConfig['advParams']['gatingParams']):
            moduleInstance['params']['gatingParams'] = moduleConfig['advParams']['gatingParams']

        if(moduleConfig['advParams']['stateParams']):
            moduleInstance['params']['stateParams'] = moduleConfig['advParams']['stateParams']

        if(moduleConfig['advParams']['unrollingParams']):
            moduleInstance['params']['unrollingParams'] = moduleConfig['advParams']['unrollingParams']

        if(moduleConfig['advParams']['sceneryParams']):
            moduleInstance['params']['sceneryParams'] = moduleConfig['advParams']['sceneryParams']

        if(moduleConfig['advParams']['presenceParams']):
            moduleInstance['params']['presenceParams'] = moduleConfig['advParams']['presenceParams']

    
    # Configure elevation angle adjustment and check if sensor is mounted on the ceiling
    thetaRot = moduleInstance['params']['sceneryParams']['sensorOrientation'][0]

    if (math.fabs(thetaRot - 90.0) < 20.5):
        moduleInstance['isCeilingMounted'] = True
    
    else:
        moduleInstance['isCeilingMounted'] = False


    if (moduleInstance['params']['presenceParams']['numOccupancyBoxes'] and moduleInstance['params']['presenceParams']['pointsThre']):
        moduleInstance['isPresenceDetectionEnabled'] = True
        moduleInstance['presenceDetectionCounter'] = 0
        moduleInstance['presenceDetectionOutput'] = 0 
        moduleInstance['presenceDetectionInitial'] = True
        moduleInstance['presenceDetectionRaw'] = False

    else:
        moduleInstance['isPresenceDetectionEnabled'] = False


    # Framerate in [ms]
    moduleInstance['params']['deltaT'] = moduleConfig['deltaT']
    dt = moduleConfig['deltaT']
    dt2 = pow(dt, 2)
    dt3 = pow(dt, 3)
    dt4 = pow(dt, 4)


    # Maximum expected target acceleration in lateral (X), longitudinal (Y), and vertical (Z) directions in [m/s2]
    # Used to compute processing noise matrix. For 2D options, the vertical component is ignored.
    moduleInstance['params']['maxAcceleration'] = moduleConfig['maxAcceleration']

    # Initialize process variance to 1/2 of maximum target acceleration
    varX = pow(0.5*moduleConfig['maxAcceleration'][0], 2)
    varY = pow(0.5*moduleConfig['maxAcceleration'][1], 2)
    varZ = pow(0.5*moduleConfig['maxAcceleration'][2], 2)

    # State vector type
    # 2DA, S={X,Y, Vx,Vy, Ax,Ay} 
    # 3DA, S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az}.
    if (moduleConfig['stateVectorType'] == STATE_VECTORS_2DA):

        # Transition matrix F for 2D space
        F6 = [
            1.0, 0.0, dt,  0.0, dt2/2, 0.0,
            0.0, 1.0, 0.0, dt,  0.0,   dt2/2,
            0.0, 0.0, 1.0, 0.0, dt,    0.0,
            0.0, 0.0, 0.0, 1.0,	0.0,   dt,
            0.0, 0.0, 0.0, 0.0, 0.0,   1.0,
        ]

        # Process noise matrix Q for 2D space
        Q6 = [
            dt4/4*varX,	0.0,        dt3/2*varX, 0.0,        dt2/2*varX,	0.0,
            0.0,        dt4/4*varY,	0.0,        dt3/2*varY,	0.0,        dt2/2*varY,
            dt3/2*varX,	0.0,        dt2*varX,	0.0,        dt*varX,    0.0,
            0.0,        dt3/2*varY,	0.0,        dt2*varY,	0.0,        dt*varY,
            dt2/2*varX,	0.0,        dt*varX,    0.0,        1.0*varX,   0.0,
            0.0,        dt2/2*varY, 0.0,        dt*varY,    0.0,        1.0*varY
        ]

        moduleInstance['params']['F'] = F6
        moduleInstance['params']['Q'] = Q6

        moduleInstance['params']['transformParams']['transformationRequired'] = False


    elif (moduleConfig['stateVectorType'] == STATE_VECTORS_3DA):

        # Transition matrix F for 3D space
        F9 = [
            1.0, 0.0, 0.0, dt, 0.0,  0.0, dt2/2, 0.0,   0.0,
            0.0, 1.0, 0.0, 0.0, dt,  0.0, 0.0,   dt2/2, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt,  0.0,   0.0,   dt2/2,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,    0.0,   0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,   dt,    0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,   0.0,   dt,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,   0.0,   0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   1.0,   0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0,   1.0,
        ]
        
        # Process noise matrix Q for 3D space
        Q9 = [
            dt4/4*varX,	0.0,        0.0,        dt3/2*varX, 0.0,        0.0,        dt2/2*varX,	0.0,        0.0,
            0.0,        dt4/4*varY,	0.0,        0.0,        dt3/2*varY, 0.0,        0.0,        dt2/2*varY,	0.0,
            0.0,    	0.0,        dt4/4*varZ,	0.0,        0.0,        dt3/2*varZ, 0.0,        0.0,        dt2/2*varZ,
            dt3/2*varX,	0.0,        0.0,        dt2*varX,	0.0,        0.0,        dt*varX,    0.0,        0.0,
            0.0,        dt3/2*varY,	0.0,        0.0,        dt2*varY,	0.0,        0.0,        dt*varY,    0.0,
            0.0,        0.0,        dt3/2*varZ,	0.0,        0.0,        dt2*varZ,	0.0,        0.0,        dt*varZ,
            dt2/2*varX,	0.0,        0.0,        dt*varX,    0.0,        0.0,        1.0*varX,   0.0,        0.0,
            0.0,        dt2/2*varY,	0.0,        0.0,        dt*varY,    0.0,        0.0,        1.0*varY,   0.0,
            0.0,        0.0,        dt2/2*varZ,	0.0,        0.0,        dt*varZ,    0.0,        0.0,        1.0*varZ,
        ]

        moduleInstance['params']['F'] = F9
        moduleInstance['params']['Q'] = Q9

        # Transformation parameters for transformation of detected points coordinater from sensor cartesian space to global cartesian space
        moduleInstance['params']['transformParams']['transformationRequired'] = True

        # Calculate sine and cosine values of theta
        # and set global space rotation
        moduleInstance['params']['transformParams']['rotX'][0] = math.sin(math.radians(thetaRot))
        moduleInstance['params']['transformParams']['rotX'][1] = math.cos(math.radians(thetaRot))

        # Set vertical offset from ground value
        moduleInstance['params']['transformParams']['offsetZ'] = moduleInstance['params']['sceneryParams']['sensorPosition'][2]


    else:
        # exit and delete module instance
        # gtrack_delete(moduleInstance)
        raise Exception('Invalid argument or unsupported state vector option.')
        

    moduleInstance['params']['stateVectorType'] = moduleConfig['stateVectorType']                       # Set state vector type

    moduleInstance['params']['maxRadialVelocity'] = moduleConfig['maxRadialVelocity']                   # Maximum radial velocity reported by sensor +/- m/s.
    moduleInstance['params']['radialVelocityResolution'] = moduleConfig['radialVelocityResolution']     # Radial Velocity resolution, m/s.
    moduleInstance['params']['initialRadialVelocity'] = moduleConfig['initialRadialVelocity']           # Expected target radial velocity at the moment of detection, m/s.


    # TODO Verboseness level
    # gtrack_log() function
    
    moduleInstance['verbose'] = moduleInstance['params']['verbose']

    # docs/doxygen3D/html/struct_gtrack_unit_instance.html
    moduleInstance['hTrack'] = [None] * moduleInstance['maxNumTracks']                                  # List of unitInstances
    moduleInstance['bestScore'] = [0] * moduleInstance['maxNumPoints']                                  # List of best scores
    moduleInstance['bestIndex'] = [0] * moduleInstance['maxNumPoints']                                  # List of IDs of best scorers
    
    # Integer division by 2^3? Same as (maxNumPoints-1) // 8?
    # Bit array that holds the indication whether measurement point is associated to one and only one track
    moduleInstance['isUniqueIndex'] = [0] * (((moduleInstance['maxNumPoints']-1) >> 3) +1)
    
    # Bit array that holds the indication whether measurement point is associated to the static track
    moduleInstance['isStaticIndex'] = [0] * (((moduleInstance['maxNumPoints']-1) >> 3) +1)
    
    # Allocation array holds the measurement indices of allocation set under construction
    moduleInstance['allocIndexCurrent'] = [0] * moduleInstance['maxNumPoints']

    # Allocation array holds the measurement indices of allocation set stored
    moduleInstance['allicIndexStored'] = [0] * moduleInstance['maxNumPoints']

    # List of tracking IDs
    moduleInstance['uidElem'] = [None] * moduleInstance['maxNumTracks']

    moduleInstance['targetNumTotal'] = 0
    moduleInstance['targetNumCurrent'] = 0

    
    for uid in range(moduleInstance['maxNumTracks']):
        try:
            moduleInstance['uidElem'][uid] = uid
            moduleInstance['freeList'][uid] = uid

            moduleInstance['params'][f'{uid}'] = uid
            moduleInstance['hTrack'][uid] = gtrack_unitCreate(moduleInstance['params'])
    
        except Exception:
            gtrack_delete(moduleInstance)
            raise Exception(f'An error has occured while creating a unit instance with Unid Identifier UID: {uid}')


    pass

def gtrack_unitCreate(trackingParams):

    pass

def gtrack_unitStart():

    pass

def gtrack_delete(instance):
    pass


def gtrack_step(pointCloud, targetList):
    """Runs a single step of given algorithm instance (module) with input point cloud data

    Args:
        pointCloud ([type]): [description]
        targetList ([type]): [description]
    """

    # Module level functions
    # tracker_modulePredict()
    # tracker_moduleAssociate()
    # tracker_moduleAllocate()
    # tracker_moduleUpdate()
    # tracker_moduleReport()
    pass

