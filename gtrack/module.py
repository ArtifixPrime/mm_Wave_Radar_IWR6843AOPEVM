import math
import logging

from bitarray.util import int2ba, ba2int

from .constants import *
from .unit import *

# Module constants
NOMINAL_ALLOCATION_RANGE    = (6.0) # Range for allocation SNR scaling
MAX_SNR_RANGE               = (2.5) # Range for SNR maximum
FIXED_SNR_RANGE             = (1.0) # Range for SNR fixed
MAX_SNR_RATIO               = ((NOMINAL_ALLOCATION_RANGE)/(MAX_SNR_RANGE))
MAX_SNR_RATIO_P4            = (MAX_SNR_RATIO)*(MAX_SNR_RATIO)*(MAX_SNR_RATIO)*(MAX_SNR_RATIO)


log = logging.getLogger('gtrack.module')


def gtrack_modulePredict(moduleInstance):
    """Module level predict function. It is called by an external step function to perform a unit level Kalman filter predictions

    Args:
        moduleInstance (dict): Module instance
    """

    # List of active unit identifiers
    activeElements = moduleInstance['activeList'].copy()

    for uid in activeElements:
        
        # Raise an assertion error if unit identifier is ever larger than configured maxmimum number of tracks.
        # This should never happen.
        assert (uid <= moduleInstance['maxNumTracks']), 'Active unit identifier is larger than maximum number of tracked objects!'

        gtrack_unitPredict(moduleInstance['hTrack'][uid])


def gtrack_moduleAssociate(moduleInstance, point, num):
    """Module level association function. It is called by external step function to associate measurement points with known targets

    Args:
        moduleInstance (dict): Module instance
        point (list): List of input measurement points. Each measurement point has range, angle and radial velocity information
        num (int): Number of input measurements (size of list)
    """

    activeElements = moduleInstance['activeList'].copy()

    for uid in activeElements:

        gtrack_unitScore(
            moduleInstance['hTrack'][uid],
            point,
            moduleInstance['bestScore'],
            moduleInstance['bestIndex'],
            moduleInstance['isUniqueIndex'],
            moduleInstance['isStaticIndex'],
            num
        )


def gtrack_moduleAllocate(moduleInstance, point, num):
    """Module level allocation function. It is called by external step function. to allocate new targets for the non-associated measurements points.

    Args:
        moduleInstance (dict): Module instance.
        point (2D list): Set of input measurements. Each measurement has information about range, angle and radial velocity.
        num (int): nNumber of input measurements.
    """

    allocationSet = {
        'isValid': False,
        'numAllocatedPoints': 0,
        'totalSNR': 0.0,
        'mCenter': []
    }

    snrThreMax = MAX_SNR_RATIO_P4 * moduleInstance['params']['allocationParams']['snrThre']
    snrThreFixed = snrThreMax/3.0
    maxAllocNum = 1
    

    for n in range(num):
        shift = n>>3
        n16 = int2ba(n,length=16, signed=False)
        barr7 = int2ba(0x0007,length=16, signed=False)
        shiftOne = ba2int(n16 & barr7)
        one8 = int2ba(1,length=8, signed=False)
        mask = one8 << shiftOne

        isPointUnique = (moduleInstance['isUniqueIndex'][shift:shift+8] & mask)


        # Allocation procedure is for points NOT associated or for the unique points that are associated to the ghost behind the target further away
        # NOTE: Points that are LIKELY ghosts (associated to the ghost due to single multipath) are excluded from the association
        if((moduleInstance['bestIndex'][n] == GTRACK_ID_POINT_NOT_ASSOCIATED) or                    # Allocation is for points NOT associated, OR
            ((moduleInstance['bestIndex'][n] == GTRACK_ID_GHOST_POINT_BEHIND) and isPointUnique)):  # Unique Point is associated to the ghost behind the target further away

            if (math.fabs(point[n][3]) < FLT_EPSILON):
                continue

            tElemFree = moduleInstance['freeList'][0]

            if tElemFree == 0:
                if log.isEnabledFor(logging.WARNING):
                    log_string = 'Maximum number of tracks reached!'
                    log.warning(log_string)

                return

            moduleInstance['allocIndexCurrent'][0] = n
            allocNum = 1
            allocSNR = point[n][4]

            mCenter = point[n][:4]
            mSum = point[n][:4]


            for k in range(n+1, num):

                shift = k>>3
                k16 = int2ba(k,length=16, signed=False)
                barr7 = int2ba(0x0007,length=16, signed=False)
                shiftOne = ba2int(k16 & barr7)
                one8 = int2ba(1,length=8, signed=False)
                mask = one8 << shiftOne

                isPointkUnique = (moduleInstance['isUniqueIndex'][shift:shift+8] & mask)

                if((moduleInstance['bestIndex'][k] == GTRACK_ID_POINT_NOT_ASSOCIATED) or                     # Allocation is for points NOT associated, OR
                    ((moduleInstance['bestIndex'][k] == GTRACK_ID_GHOST_POINT_BEHIND) and isPointkUnique)):  # Unique Point is associated to the ghost behind the target further away

                    if (math.fabs(point[k][3]) < FLT_EPSILON):
                        continue

                    mCurrent = point[k][:4]
                    mCurrent[3] = gtrack_unrollRadialVelocity(moduleInstance['params']['maxRadialVelocity'], mCenter[3], mCurrent[3])

                    if (math.fabs(mCurrent[3] - mCenter[3]) < moduleInstance['params']['allocationParams']['maxVelThre']):

                        dist = gtrack_calcDistance(mCenter, mCurrent)
                        
                        if (math.sqrt(dist) < moduleInstance['params']['allocationParams']['maxDistanceThre']):

                            moduleInstance['allocIndexCurrent'][allocNum] = k

                            allocNum += 1
                            allocSNR += point[k][4]

                            # Update centroid
                            mSum = mCurrent + mSum
                            factor = 1.0/float(allocNum)
                            mCenter = mSum * factor


            if ((allocNum > maxAllocNum) and (math.fabs(mCenter[3]) >= moduleInstance['params']['allocationParams']['velocityThre'])):
                maxAllocNum = allocNum
                allocationSet['isValid'] = True
                allocationSet['numAllocatedPoints'] = allocNum
                allocationSet['mCenter'] = mCenter
                allocationSet['totalSNR'] = allocSNR

                for k in range(allocNum):
                    moduleInstance['allocIndexStored'].insert(k, moduleInstance['allocIndexCurrent'][k])

    # Presence detection
    if (moduleInstance['isPresenceDetectionEnabled'] and allocationSet['isValid'] and (moduleInstance['presenceDetectionRaw'] == 0)):

        # If presence detection enabled AND we have a valid set and haven't detected yet => proceed with presence detection
        if(
            (allocationSet['numAllocatedPoints'] >= moduleInstance['params']['presenceParams']['pointsThre']) and
            (moduleInstance['isPresenceDetectionInitial'] or (allocationSet['mCenter'][3] <= -moduleInstance['params']['presenceParams']['velocityThre']))):

            mPos = gtrack_sph2cart(allocationSet['mCenter'])

            for box in range(moduleInstance['params']['presenceParams']['numOccupancyBoxes']):
                if (gtrack_isPointInsideBox(mPos, moduleInstance['params']['presenceParams']['occupancyBox'][box])):

                    log_string3D = f"{moduleInstance['heartBeat']}: Presence set {allocationSet['numAllocatedPoints']} points, doppler = {allocationSet['mCenter'][3]}, at ({mPos[0]}, {mPos[1]}, {mPos[2]})\n"
                    #log_string2D = f"{moduleInstance['heartBeat']}: Presence set {allocationSet['numAllocatedPoints']} points, doppler = {allocationSet['mCenter'][]}, at ({mPos[0]}, {mPos[1]})\n"
                    log.debug(log_string3D)

                    moduleInstance['presenceDetectionRaw'] = 1
                    break

    if allocationSet['isValid']:
        if (
            (allocationSet['numAllocatedPoints'] >= moduleInstance['params']['allocationParams']['pointsThre']) and
            (math.fabs(allocationSet['mCenter'][3]) >= moduleInstance['params']['allocationParams']['velocityThre'])):

            isBehind = False
            activeElements = moduleInstance['activeList'].copy()

            for uid in activeElements:

                activeUnitInstance = moduleInstance['hTrack'][uid]
                uCenter = activeUnitInstance['uCenter']
                spread = activeUnitInstance['estSpread']

                if gtrack_isPointBehindTarget(allocationSet['mCenter'], uCenter, spread):

                    isBehind = True
                    break

            
            if(allocationSet['mCenter'][0] < FIXED_SNR_RANGE):
                snrThreshold = snrThreFixed

            else:
                snrRatio = NOMINAL_ALLOCATION_RANGE/allocationSet['mCenter'][0]
                snrRatio4 = snrRatio*snrRatio*snrRatio*snrRatio

                if isBehind:
                    snrThreshold = snrRatio4*moduleInstance['params']['allocationParams']['snrThreObscured']

                else:
                    if(allocationSet['mCenter'][0] < MAX_SNR_RANGE):
                        snrThreshold = allocationSet['mCenter'][0] * (snrThreMax - snrThreFixed) / (MAX_SNR_RANGE - FIXED_SNR_RANGE)

                    else:
                        snrThreshold = snrRatio4*moduleInstance['params']['allocationParams']['snrThre']

            
            if (log.isEnabledFor(logging.DEBUG)):
                coordinates = f''
                for coord in range(len(allocationSet['mCenter'])):
                    coordinates += f"{allocationSet['mCenter'][coord]}, "

                log_string = f"{moduleInstance['heartBeat']}:Allocation set {allocationSet['numAllocatedPoints']} points, centroid at [{coordinates}], total SNR {allocationSet['totalSNR']} > {snrThreshold}\n"
                log.debug(log_string)

            
            if (allocationSet['totalSNR'] > snrThreshold):
                # Associate points with new UID
                for k in range(allocationSet['numAllocatedPoints']):

                    # Temporary Array of measurement indices for stored set-under-construction
                    allocIndex = moduleInstance['allocIndexStored'][k]
                    moduleInstance['bestIndex'][allocIndex] = int(tElemFree)


                # Allocate new tracker
                moduleInstance['targetNumTotal'] += 1
                moduleInstance['targetNumCurrent'] += 1

                # Remove the first element (unit instance with uid) from the list of free tracking units and store it
                tElemFree = moduleInstance['freeList'].pop(0)

                gtrack_unitStart(
                    moduleInstance['hTrack'][tElemFree],
                    moduleInstance['heartBeat'],
                    moduleInstance['targetNumTotal'],
                    allocationSet['mCenter'],
                    isBehind
                )

                # Move the tracking unit to the list of active tracking units
                moduleInstance['activeList'].append(tElemFree)


def gtrack_moduleUpdate(moduleInstance, point, num, var = None):
    """Module level update function. It is called by external step function to perform unit level kalman filter updates.

    Args:
        moduleInstance (dict): Module instance.
        point (2D list): Set of input measurements. Each measurement has the information about range, angle and radial velocity.
        num (int): Number of input measurements.
        var (2D list, optional): Set of input measurement variances. Defaults to None if variances are unknown.
    """

    activeList = moduleInstance['activeList'].copy()

    for uid in activeList:

        state = gtrack_unitUpdate(moduleInstance['hTrack'][uid], point, moduleInstance['bestIndex'],  moduleInstance['isUniqueIndex'], num, var)

        if (state == TRACK_STATE_FREE):
            tElemToRemove = uid
            moduleInstance['activeList'].remove(tElemToRemove)

            # Unit stop
            # Stop target tracking when tracking state transitions to FREE
            freedUnitInstance = moduleInstance['hTrack'][uid]
            log_string = f"{freedUnitInstance['heartBeatCount']}: uid[{freedUnitInstance['uid']}] FREED: S=[{freedUnitInstance['S_hat'][0]}, {freedUnitInstance['S_hat'][1]}, {freedUnitInstance['S_hat'][2]}, {freedUnitInstance['S_hat'][3]}]\n"
            log.debug(log_string)

            moduleInstance['hTrack'][uid] = None

            moduleInstance['targetNumCurrent'] -=1
            moduleInstance['freeList'].append(tElemToRemove)


def gtrack_modulePresence(moduleInstance, presence):
    """Module level presence detection function. It is called by external step function

    Args:
        moduleInstance (dict): Module instance
        presence (int): Presence indicator

    Returns:
        bool: Presence indicator
    """


    oldPresence = moduleInstance['presenceDetectionOutput']

    # Presence can be detected at allocation time (i.e. presenceDetectionRaw can be set there), or
    # it can be set below based on existing tracks
    if moduleInstance['isPresenceDetectionEnabled']:
        if (moduleInstance['presenceDetectionRaw'] == 0):

            activeList = moduleInstance['activeList'].copy()
            for uid in activeList:
                activeUnitInstance = moduleInstance['hTrack'][uid]

                if moduleInstance['params']['transformParams']['transformationRequired']:
                    # Get centroid of associated measurement points in cartesian form
                    mCenter = activeUnitInstance['uPos']
                    posW = gtrack_censor2world(mCenter, moduleInstance['params']['transformParams'])

                else:
                    posW = activeUnitInstance['uPos']

                
                for numBoxes in range(moduleInstance['params']['presenceParams']['numOccupancyBoxes']):
                    if gtrack_isPointInsideBox(posW, moduleInstance['params']['presenceParams']['occupancyBox'][numBoxes]):
                        moduleInstance['presenceDetectionRaw'] = 1
                        break

        
    if moduleInstance['presenceDetectionRaw']:
        moduleInstance['presenceDetectionOutput'] = 1
        moduleInstance['presenceDetectionCounter'] = 0

    elif (moduleInstance['presenceDetectionOutput'] == 1):
        moduleInstance['presenceDetectionCounter'] +=1

        log_string = f"{moduleInstance['heartBeat']}: Presence ON, Counter {moduleInstance['presenceDetectionCounter']}\n"
        log.debug(log_string)

        
        if (moduleInstance['presenceDetectionCounter'] >= moduleInstance['params']['presenceParams']['on2offThre']):
            moduleInstance['isPresenceDetectionInitial'] = False
            moduleInstance['presenceDetectionOutput'] = 0
            moduleInstance['presenceDetectionCounter'] = 0

    
    if (oldPresence != moduleInstance['presenceDetectionOutput']):
        if (moduleInstance['presenceDetectionOutput'] == 0):
            log_string = f"{moduleInstance['heartBeat']}: Presence OFF\n"
            log.debug(log_string)

        else:
            log_string = f"{moduleInstance['heartBeat']}: Presence ON\n"
            log.debug(log_string)


    if (presence != 0):
        presence = moduleInstance['presenceDetectionOutput']

    return presence


def gtrack_moduleReport(moduleInstance, t):
    """Module level report function. It is called by external step function to obtain unit level data.

    Args:
        moduleInstance (dict): Module instance
        t (list): List of target descriptors which this function populates. Target descriptors are dictionaries.

    Returns:
        int: Number of populated target descriptors.
    """

    tNum = 0
    activeList = moduleInstance['activeList'].copy()

    for uid in activeList:

        gtrack_unitReport(moduleInstance['hTrack'][uid], t, tNum)
        tNum +=1

    return tNum
