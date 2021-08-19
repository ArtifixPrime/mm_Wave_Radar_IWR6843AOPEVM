# imports
import math
import numpy as np
import time
import logging

from bitarray import bitarray
from .constants import *
from .utilities import *
from .module import *


log = logging.getLogger(__name__)


def gtrack_unitCreate(trackingParams):
    """Instantiate a gtrack unit with desired configuration parameters (inherited from higher level module).

    Args:
        trackingParams (dict): Configuration structure passed from parent module instance. The structure contains all parameters that are axposed by gtrack algorithm.
        The configuration does not need to persist.

    Raises:
        ValueError: State vector type must be set for motion model with contant acceleration.

    Returns:
        dict: Structure describing a tracking unit (unit instance).
    """

    # Unit instance initialization
    # docs/doxygen3D/html/struct_gtrack_unit_instance.html
    unitInstance = unitInstanceSch.deepcopy()

    # Setting parameters
    unitInstance['params']['gatingParams'] = trackingParams['gatingParams']
    unitInstance['params']['stateParams'] = trackingParams['stateParams']
    unitInstance['params']['allocationParams'] = trackingParams['allocationParams']
    unitInstance['params']['unrollingParams'] = trackingParams['unrollingParams']
    unitInstance['params']['sceneryParams'] = trackingParams['sceneryParams']
    unitInstance['params']['transformParams'] = trackingParams['transformParams']
    
    unitInstance['maxAcceleration'] = trackingParams['maxAcceleration']
    
    unitInstance['uid'] = trackingParams['uid']
    unitInstance['isTargetStatic'] = False
    unitInstance['maxRadialVelocity'] = trackingParams['maxRadialVelocity']
    unitInstance['initialRadialVelocity'] = trackingParams['initialRadialVelocity']
    unitInstance['radialVelocityResolution'] = trackingParams['radialVelocityResolution']
    unitInstance['verbose'] = trackingParams['verbose']

    unitInstance['params']['F'] = trackingParams['F']
    unitInstance['params']['Q'] = trackingParams['Q']

    stateVectorType = trackingParams['stateVectorType']

    if stateVectorType == STATE_VECTORS_2DA:
        unitInstance['stateVectorType'] = STATE_VECTORS_2DA
        unitInstance['stateVectorDimNum'] = 2
        unitInstance['stateVectorDimLength'] = 3
        unitInstance['stateVectorLength'] = 6
        unitInstance['measurementVectorLength'] = 3

        unitInstance['minStaticVelocityOne'] = 0.1
        unitInstance['minStaticVelocityTwo'] = 0.1
        unitInstance['isAssociationGhostMarking'] = True
        unitInstance['isAssociationHeightIgnore'] = False

        unitInstance['isEnablePointNumberEstimation'] = True
        unitInstance['isSnrWeighting'] = True
        unitInstance['estNumOfPoints'] = unitInstance['params']['allocationParams']['pointsThre']
        unitInstance['estSpreadAlpha'] = 0.05
        unitInstance['confidenceLevelThreshold'] = 0.7

    elif stateVectorType == STATE_VECTORS_3DA:
        unitInstance['stateVectorType'] = STATE_VECTORS_3DA
        unitInstance['stateVectorDimNum'] = 3
        unitInstance['stateVectorDimLength'] = 3
        unitInstance['stateVectorLength'] = 9
        unitInstance['measurementVectorLength'] = 4

        unitInstance['minStaticVelocityOne'] = 0.5
        unitInstance['minStaticVelocityTwo'] = 0.5
        unitInstance['isAssociationGhostMarking'] = False
        unitInstance['estSpreadAlpha'] = 0.01

        elevTilt = unitInstance['params']['sceneryParams']['sensorOrientation'][1]

        if (math.fabs(90.0 - elevTilt) < 20.5):
            # Sensor is ceiling mounted
            unitInstance['isAssociationHeightIgnore'] = True
            unitInstance['isEnablePointNumberEstimation'] = False
            unitInstance['isSnrWeighting'] = False
            unitInstance['estNumOfPoints'] = 100.0
            unitInstance['confidenceLevelThreshold'] = 0.3

        else:
            # Sensor is wall mounted
            unitInstance['isAssociationHeightIgnore'] = False
            unitInstance['isEnablePointNumberEstimation'] = True
            unitInstance['isSnrWeighting'] = True
            unitInstance['estNumOfPoints'] = unitInstance['params']['allocationParams']['pointsThre']
            unitInstance['confidenceLevelThreshold'] = 0.5

    else:
        raise ValueError('Invalid argument or unsupported state vector option.')


    unitInstance['dt'] = trackingParams['deltaT']
    unitInstance['state'] = TRACK_STATE_FREE

    return unitInstance


def gtrack_delete(objectType, activeModules):
    """A function that removes module instance from the list of active module instances

    Args:
        objectType (str):     Name of module instance
        activeModules (dict): Dictionary of actice module instances
    """

    activeModules.pop(objectType)


def gtrack_create(moduleConfig, objectType, activeModules):
    """Creates a module instance with parameter values that are constant for each of the tracked units within the module.
    These parameters are:
        a) Default parameters for gating, statem velocity unrolling, allocation and presence.
        b) Parameters for wall-mount or ceiling-mount configuration of the sensor (based on height and elevation tilt).
        c) Initialize Process Noise Matrix Q and State transition Matrix F.

    Args:
        moduleConfig (dict): Contains the parameters that are exposed by gtrack algorithm. The configuration does not need to persist.
        Advanced configuration structure can be set to NULL to use the default one. Any field within Advanced configuration can also be set to NULL 
        to use the default values for the field.
        objectType (str): Name of the type of object that the instance will be tracking.
        activeModules (dict): List of active modules that stores the instance after initialization

    Raises:
        TypeError: If objectType is not a string.
        ValueError: If configured number of measurement points exceeds maximum allowed number of points.
        ValueError: If configured number of tracking targets exceeds maximum allowed number of tracking targets.
        ValueError: Unsupported state vector type. Only motion models with contant acceleration are supported.
        Exception: Raised in case gtrack_unitCreate() encounters a problem.
    """

    if not isinstance(objectType, str):
        raise TypeError('objectType must be a string.')


    # Exceptions
    if(moduleConfig['maxNumPoints'] > GTRACK_NUM_POINTS_MAX):
        raise ValueError('Invalid argument or the configured number of measurement points exceeds the maximum allowed number of measurement points.')
        

    if(moduleConfig['maxNumTracks'] > GTRACK_NUM_TRACKS_MAX):
        raise ValueError('Invalid argument or the configured number of tracking targets exceeds the maximum allowed number of tracking targets.')


    # Module instance initialization
    # docs/doxygen3D/html/struct_gtrack_module_instance.html
    moduleInstance = moduleInstanceSch.deepcopy()

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
    thetaRot = moduleInstance['params']['sceneryParams']['sensorOrientation'][1]

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
    moduleConfig['maxAcceleration'] = np.zeros(3, dtype=float)
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
        F6 = np.array([
            [1.0, 0.0, dt,  0.0, dt2/2, 0.0  ],
            [0.0, 1.0, 0.0, dt,  0.0,   dt2/2],
            [0.0, 0.0, 1.0, 0.0, dt,    0.0  ],
            [0.0, 0.0, 0.0, 1.0, 0.0,   dt   ],
            [0.0, 0.0, 0.0, 0.0, 0.0,   1.0, ]
        ], dtype=float)

        # Process noise matrix Q for 2D space
        Q6 = np.array([
            [dt4/4*varX,    0.0,        dt3/2*varX, 0.0,        dt2/2*varX, 0.0       ],
            [0.0,           dt4/4*varY, 0.0,        dt3/2*varY, 0.0,        dt2/2*varY],
            [dt3/2*varX,    0.0,        dt2*varX,   0.0,        dt*varX,    0.0       ],
            [0.0,           dt3/2*varY, 0.0,        dt2*varY,   0.0,        dt*varY   ],
            [dt2/2*varX,    0.0,        dt*varX,    0.0,        1.0*varX,   0.0       ],
            [0.0,           dt2/2*varY, 0.0,        dt*varY,    0.0,        1.0*varY  ]
        ], dtype=float)

        moduleInstance['params']['F'] = F6
        moduleInstance['params']['Q'] = Q6

        moduleInstance['params']['transformParams']['transformationRequired'] = False


    elif (moduleConfig['stateVectorType'] == STATE_VECTORS_3DA):

        # Transition matrix F for 3D space
        F9 = np.array([
            [1.0, 0.0, 0.0, dt, 0.0,  0.0, dt2/2, 0.0,   0.0  ],
            [0.0, 1.0, 0.0, 0.0, dt,  0.0, 0.0,   dt2/2, 0.0  ],
            [0.0, 0.0, 1.0, 0.0, 0.0, dt,  0.0,   0.0,   dt2/2],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,    0.0,   0.0  ],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,   dt,    0.0  ],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,   0.0,   dt   ],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,   0.0,   0.0  ],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   1.0,   0.0  ],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0,   1.0  ]
        ], dtype=float)
        
        # Process noise matrix Q for 3D space
        Q9 = np.array([
            [dt4/4*varX, 0.0,        0.0,        dt3/2*varX, 0.0,        0.0,        dt2/2*varX, 0.0,        0.0     ],
            [0.0,        dt4/4*varY, 0.0,        0.0,        dt3/2*varY, 0.0,        0.0,        dt2/2*varY, 0.0     ],
            [0.0,        0.0,        dt4/4*varZ, 0.0,        0.0,        dt3/2*varZ, 0.0,        0.0,        dt2/2*varZ  ],
            [dt3/2*varX, 0.0,        0.0,        dt2*varX,   0.0,        0.0,        dt*varX,    0.0,        0.0     ],
            [0.0,        dt3/2*varY, 0.0,        0.0,        dt2*varY,   0.0,        0.0,        dt*varY,    0.0     ],
            [0.0,        0.0,        dt3/2*varZ, 0.0,        0.0,        dt2*varZ,   0.0,        0.0,        dt*varZ ],
            [dt2/2*varX, 0.0,        0.0,        dt*varX,    0.0,        0.0,        1.0*varX,   0.0,        0.0     ],
            [0.0,        dt2/2*varY, 0.0,        0.0,        dt*varY,    0.0,        0.0,        1.0*varY,   0.0     ],
            [0.0,        0.0,        dt2/2*varZ, 0.0,        0.0,        dt*varZ,    0.0,        0.0,        1.0*varZ]
        ], dtype=float)

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
        raise ValueError('Invalid argument or unsupported state vector option.')
        

    moduleInstance['params']['stateVectorType'] = moduleConfig['stateVectorType']                       # Set state vector type

    moduleInstance['params']['maxRadialVelocity'] = moduleConfig['maxRadialVelocity']                   # Maximum radial velocity reported by sensor +/- m/s.
    moduleInstance['params']['radialVelocityResolution'] = moduleConfig['radialVelocityResolution']     # Radial Velocity resolution, m/s.
    moduleInstance['params']['initialRadialVelocity'] = moduleConfig['initialRadialVelocity']           # Expected target radial velocity at the moment of detection, m/s.


    # TODO Verboseness level
    # gtrack_log() function
    
    moduleInstance['verbose'] = moduleInstance['params']['verbose']

    # docs/doxygen3D/html/struct_gtrack_unit_instance.html
    moduleInstance['hTrack'] = [None] * moduleInstance['maxNumTracks']                                  # List of unitInstances
    moduleInstance['bestScore'] = np.zeros(moduleInstance['maxNumPoints'], dtype=int)                   # List of best scores
    moduleInstance['bestIndex'] = np.zeros(moduleInstance['maxNumPoints'], dtype=int)                   # List of IDs of best scorers
    
    # Integer division by 2^3? Same as (maxNumPoints-1) // 8?
    # Bit array that holds the indication whether measurement point is associated to one and only one track
    # moduleInstance['isUniqueIndex'] = np.zeros((((moduleInstance['maxNumPoints']-1) >> 3) +1)*8, dtype='uint8')
    moduleInstance['isUniqueIndex'] = bitarray((((moduleInstance['maxNumPoints']-1) >> 3) +1)*8)
    moduleInstance['isUniqueIndex'].setall(0)
    
    # Bit array that holds the indication whether measurement point is associated to the static track
    # moduleInstance['isStaticIndex'] = np.zeros((((moduleInstance['maxNumPoints']-1) >> 3) +1)*8, dtype='uint8')
    moduleInstance['isStaticIndex'] = bitarray((((moduleInstance['maxNumPoints']-1) >> 3) +1)*8)
    moduleInstance['isStaticIndex'].setall(0)
    
    # Allocation array holds the measurement indices of allocation set under construction
    moduleInstance['allocIndexCurrent'] = np.zeros(moduleInstance['maxNumPoints'], dtype=int)

    # Allocation array holds the measurement indices of allocation set stored
    moduleInstance['allicIndexStored'] = np.zeros(moduleInstance['maxNumPoints'], dtype=int)

    # List of tracking IDs
    moduleInstance['uidElem'] = [None] * moduleInstance['maxNumTracks']

    moduleInstance['targetNumTotal'] = 0
    moduleInstance['targetNumCurrent'] = 0

    
    for uid in range(moduleInstance['maxNumTracks']):
        try:
            moduleInstance['uidElem'][uid] = uid
            moduleInstance['freeList'].append(moduleInstance['uidElem'][uid])

            moduleInstance['params']['uid'] = uid
            moduleInstance['hTrack'][uid] = gtrack_unitCreate(moduleInstance['params'])
    
        except Exception:
            raise Exception(f'An error has occured while creating a unit instance with Unit Identifier UID: {uid}')


    activeModules[objectType] = moduleInstance


def gtrack_step(moduleInstance, point, mNum, t, tNum, mIndex, uIndex, presence, bench = None, var = None):
    """Runs a single step of given algorithm instance (module) with input point cloud data. 
    Process one frame of measurements with a given instance of the gtrack algorithm.

    Args:
        moduleInstance (dict): Module instance
        point (2D-list): Point cloud of input measurements in form of [range, azimuth, elevation, velocity, snr]
        mNum (int): Number of input measurements.
        var (list, optional): List of input measurement variances. Defaults to None if variances are unknown.

    Returns:
        t (list): List of target descriptor
        tNum (int): Number of populated target descriptors
        mIndex (int): [description]
        uIndex ([type]): [description]
        presence (bool): Presence indicator. If configured, this function returns presence indication, FALSE if occupancy area is empty and TRUE if occupied
        bench (list): Benchmarking results. Each result is a timestamp. This function populates the list with with the timestamps which are generated with time.monotonic()
        Defaults to None when benchmarking isn't required.
    """


    moduleInstance['heartBeat'] +=1
    moduleInstance['presenceDetectionRaw'] = 0

    if (mNum > moduleInstance['maxNumPoints']):
        mNum = moduleInstance['maxNumPoints']

        arrSize = (moduleInstance['maxNumPoints']>>3) + 1
        moduleInstance['isUniqueIndex'] = bitarray('1111 1111')*arrSize
        moduleInstance['isStaticIndex'] = bitarray('0000 0000')*arrSize

        for n in range(mNum):
            moduleInstance['bestScore'][n] = FLT_MAX

            # if GTRACK_3D
            # If in ceiling mount configuration, ignore points at direct boresight
            if moduleInstance['isCeilingMounted']:
                if gtrack_isInsideBoresightStaticZone(point[n]):
                    moduleInstance['bestIndex'][n] = GTRACK_ID_POINT_BEHIND_THE_WALL
                    continue

            
            if moduleInstance['params']['transormParams']['transformationRequired']:
                pos = gtrack_sph2cart(point[n])
                posW = gtrack_censor2world(pos, moduleInstance['params']['transormParams'])

            else:
                posW = gtrack_sph2cart(point[n])

            
            for numBoundaryBoxes in range(moduleInstance['params']['sceneryParams']['numBoundaryBoxes']):
                if gtrack_isPointInsideBox(posW, moduleInstance['params']['sceneryParams']['BoundaryBox'][numBoundaryBoxes]):
                    # Inside boundary box
                    if (math.fabs(point[n][3]) > FLT_MIN):
                        # Valid dynamic point
                        moduleInstance['bestIndex'][n] = GTRACK_ID_POINT_NOT_ASSOCIATED

                    else:
                        # Additional check for static points
                        if moduleInstance['params']['sceneryParams']['numStaticBoxes']:
                            for numStaticBoxes in range(moduleInstance['params']['sceneryParams']['numStaticBoxes']):
                                if gtrack_isPointInsideBox(posW, moduleInstance['params']['sceneryParams']['StaticBox'][numStaticBoxes]):
                                    # Valid static point
                                    moduleInstance['bestIndex'][n] = GTRACK_ID_POINT_NOT_ASSOCIATED
                                    break


                        else:
                            # No static boxes, hence static point is valid
                            moduleInstance['bestIndex'][n] = GTRACK_ID_POINT_NOT_ASSOCIATED
                    

                    break
        
        else:
            # No boundaries, hence point is valid
            moduleInstance['bestIndex'][n] = GTRACK_ID_POINT_NOT_ASSOCIATED

    # Module level functions
    if bench is not None:
        # Collect benchmarks
        bench[GTRACK_BENCHMARK_SETUP] = time.monotonic()
        gtrack_modulePredict(moduleInstance)

        bench[GTRACK_BENCHMARK_PREDICT] = time.monotonic()
        gtrack_moduleAssociate(moduleInstance, point, mNum)

        bench[GTRACK_BENCHMARK_ASSOCIATE] = time.monotonic()
        gtrack_moduleAllocate(moduleInstance, point, mNum)

        bench[GTRACK_BENCHMARK_ALLOCATE] = time.monotonic()
        gtrack_moduleUpdate(moduleInstance, point, var, mNum)

        bench[GTRACK_BENCHMARK_UPDATE] = time.monotonic()
        presence = gtrack_modulePresence(moduleInstance, presence)

        bench[GTRACK_BENCHMARK_PRESENCE] = time.monotonic()
        tNum = gtrack_moduleReport(moduleInstance, t)

        bench[GTRACK_BENCHMARK_REPORT] = time.monotonic()

    else:
        gtrack_modulePredict(moduleInstance)
        gtrack_moduleAssociate(moduleInstance, point, mNum)
        gtrack_moduleAllocate(moduleInstance, point, mNum)
        gtrack_moduleUpdate(moduleInstance, point, var, mNum)
        presence = gtrack_modulePresence(moduleInstance, presence)
        tNum = gtrack_moduleReport(moduleInstance, t)


    # If requested, report unique bitmap
    if (uIndex != 0):
        if mNum:
            for n in range(((mNum-1)>>3)+1):
                uIndex[n] = moduleInstance['isUniqueIndex'][n]

    
    # If requested, report UIDs associated with measurement vector
    if (mIndex != 0):
        for n in range(mNum):
            mIndex[n] = moduleInstance['bestIndex'][n]


    return tNum, mIndex, uIndex, presence, bench

