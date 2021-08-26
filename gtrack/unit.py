import numpy as np
import logging
from bitarray.util import int2ba, ba2int

from .constants import *
from .utilities import *
from .utilities_3d import *


log = logging.getLogger(__name__)


def gtrack_unitPredict(unitInstance):
    """Perform unit prediction step.

    Args:
        unitInstance (dict): Active unit instance.
    """

    unitInstance['heartBeatCount'] += 1

    if (unitInstance['isTargetStatic'] or unitInstance['skipPrediction']):
        # Maintain S and P for static objects
        unitInstance['S_apriori_hat'] = unitInstance['S_hat']
        unitInstance['P_apriori_hat'] = unitInstance['P_hat']

    else:
        # S_apriori_hat = F * S_hat
        unitInstance['S_apriori_hat'] = np.matmul(unitInstance['params']['F'], unitInstance['S_hat'])
        
        if unitInstance['skipProcessNoiseEst'] == False:
            # P_apriori = F * P * F.T + Q
            tmp1 = np.matmul(unitInstance['params']['F'], unitInstance['P_hat'])
            tmp2 = np.matmul(tmp1, unitInstance['params']['F'].T)
            tmp3 = tmp2 + unitInstance['params']['Q']

            # Same as gtrack_matrixMakeSymmetrical()
            # It makes matrix tmp3 symmetrical, by keeping the diagonal 
            # while adding the two values opposite across the diagonal and averaging them
            unitInstance['P_apriori_hat'] = 0.5*(tmp3 + tmp3.T)
        
        
    unitInstance['skipPrediction'] = False
    unitInstance['skipProcessNoiseEst'] = False

    spherical = gtrack_cartesian2spherical(unitInstance['currentStateVectorType'], unitInstance['S_apriori_hat'])
    unitInstance['H_s'] = spherical

    limits = gtrack_calcMeasurementLimits(unitInstance['H_s'][0], unitInstance['params']['gatingParams']['limits'])
    unitInstance['H_limits'] = limits


    if log.isEnabledFor(logging.DEBUG):
        log_string = f'{unitInstance["heartBeatCount"]}: uid[{unitInstance["uid"]}]: Predict: S-hat => S-apriori-hat\n'
        log.debug(log_string)

        # Print side by side unitInstance['S_hat'] unitInstance['S_apriori_hat']
        for row in range(len(unitInstance['S_hat'])):
            row_S_hat = unitInstance['S_hat'][row]
            row_S_apr = unitInstance['S_apriori_hat'][row]

            log_string += f"{row_S_hat}\t|\t{row_S_apr}\n"

        log.debug(log_string)


        # Print unitInstance['P_hat']
        log_string = f'{unitInstance["heartBeatCount"]}: uid[{unitInstance["uid"]}]: Predict: P-hat, Q => P-apriori-hat\n'
        log.debug(log_string)

        log_string = np.array_str(unitInstance['P_hat']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)

        # Print unitInstance['params']['Q']
        log_string = np.array_str(unitInstance['params']['Q']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)

        # Print unitInstance['P_apriori_hat']
        log_string = np.array_str(unitInstance['P_apriori_hat']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)

        # Print unitInstance['H_s']
        log_string = f'{unitInstance["heartBeatCount"]}: uid[{unitInstance["uid"]}]: Predict: H-s =\n'
        log.debug(log_string)

        log_string = np.array_str(unitInstance['H_s']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)
    

def gtrack_unitScore(unitInstance, point, bestScore, bestIndex, isUnique, isStatic, num):
    """Module instance calls this function to obtain the measurement vector scoring from a units' perspective

    Args:
        unitInstance (dict): Module instance of active Unit UID
        point (list): List of measurement points
        bestScore (float): Scoresheet with best scores. Only better scores are updated
        bestIndex (int): Current scoresheet winners. Only better score winners are updated
        isUnique (binary array): An array indicating whether point belongs to a single target (1) or not (0)
        num (int): Number of measurement points
    """

    log_string = f'{unitInstance["heartBeatCount"]}: {unitInstance["uid"]}: Scoring: G={unitInstance["G"]}\n'
    log.debug(log_string)

    # Doppler limits
    limits = unitInstance['H_limits']
    # Add doppler agility
    limits[3] = min(2*unitInstance['H_limits'][3], 2*unitInstance['estSpread'][3])

    unitInstance['numAssosiatedPoints'] = 0


    if unitInstance['isAssociationGhostMarking']:
        # Compute single multipath ghost location
        hs_ghost = unitInstance['H_s']
        hs_ghost[0] += 1.2 # Increase range by 1.2 m


    if unitInstance['isAssociationHeightIgnore']:
        # Ignore logdet for 3D CM
        logdet = 0.0
        posHs = gtrack_sph2cart(unitInstance['H_s'])

    else:
        # logdet = math.log(unitInstance['gC_det'])
        logdet = unitInstance['gC_det'][1]


    for n in range(num):
        if (bestIndex[n] == GTRACK_ID_POINT_BEHIND_THE_WALL): # Point is not associated, is behind the wall
            continue

        if unitInstance['isAssociationHeightIgnore']:
            posU = gtrack_sph2cart(point[n])
            posHs[1] = posU[1]      # posY
            hs_projection = gtrack_cart2sph(posHs)

            u_tilda = point[n][:4] - hs_projection

        else:
            u_tilda = point[n][:4] - unitInstance['H_s']

    
        rvOut = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], unitInstance['H_s'][3], point[n][3])
        # Doppler
        u_tilda[3] = rvOut - unitInstance['H_s'][3]

        # Any point outside the limits is outside the gate
        isWithinLimits = True
        isInsideGate = False
        isInsideInnerGate = False
        isGhostPoint = False
        isDynamicPoint = math.fabs(rvOut) > FLT_EPSILON


        if unitInstance['isAssociationGhostMarking']:
            # Ghost point marking
            if(
                (unitInstance['isTargetStatic'] == False) and                       # Only moving targets can produce ghosts
                (point[n][0] > unitInstance['H_s'][0]) and                          # Behind the target
                gtrack_isInsideSolidAngle(u_tilda, unitInstance['estSpread']) and   # Only within solid angle spread
                isDynamicPoint and                                                  # Only dynamic points can be marked
                (math.fabs(u_tilda[3]) < 2*unitInstance['estSpread'][3])):          # Only similar Doppler
                
                # This is a point behind existing target, hence mark it as potential multipath reflection
                isGhostPoint = True
                tid = GTRACK_ID_GHOST_POINT_BEHIND
                score_ghost = 100.0

                # Check whether it is likely to be a single multipath reflection
                u_tilda_ghost = point[n][:4] - hs_ghost
                mahalanobis = gtrack_computeMahalanobis(u_tilda_ghost, unitInstance['gC_inv'])

                if (mahalanobis < 1.0):
                    # Most likely a ghost point
                    tid = GTRACK_ID_GHOST_POINT_LIKELY
                    score_ghost = logdet + mahalanobis

                score = score_ghost

        for m in range(len(u_tilda)):
            if (math.fabs(u_tilda[m]) > limits[m]):

                isWithinLimits = False
                break
        

        if isWithinLimits:
            
            # For gating purposes doppler information is not yet required so we need only partial Mahalanobis distance
            mahalanobisPart = gtrack_computeMahalanobisPartial(u_tilda, unitInstance['gC_inv'])

            # Gating step
            if (mahalanobisPart < unitInstance['G']):
                # Within the Gate, compute scoring function using all dimensions
                isInsideGate = True

                mahalanobis = gtrack_computeMahalanobis(u_tilda, unitInstance['gC_inv'])

                if(mahalanobis < unitInstance['G']):
                    isInsideInnerGate = True

                score_gate = logdet + mahalanobis

                if isGhostPoint:
                    if (score_gate < score_ghost):
                        score = score_gate
                        tid = unitInstance['uid']
                        isGhostPoint = False

                        if isDynamicPoint:
                            unitInstance['numAssosiatedPoints'] += 1

                else:
                    score = score_gate
                    tid = unitInstance['uid']

                    if isDynamicPoint:
                        unitInstance['numAssosiatedPoints'] += 1

        if (isInsideGate or isGhostPoint):
            if (bestIndex[n] > GTRACK_ID_GHOST_POINT_BEHIND):

                # No competition. Register the score and index.
                bestScore[n] = score
                bestIndex[n] = tid

                if unitInstance['isTargetStatic']:
                    # Set static indication
                    # isStatic[n>>3] |= (1<<(n & 0x0007))
                    shift = n>>3
                    n16 = int2ba(n,length=16, signed=False)          # 16-bit bitarray representation of n
                    barr7 = int2ba(0x0007,length=16, signed=False)   # 16-bit bitarray representation of 0x0007
                    shiftOne = ba2int(n16 & barr7)
                    one16 = int2ba(1,length=16, signed=False)        # 16-bit binary array in form of bitarray('0000000000000001')
                    mask = one16 << shiftOne
                    mask = mask[8:16]

                    isStatic[shift:shift+8] |= mask

                point[n][3] = rvOut # doppler

            else:
                # There is a competitor
                clearUniqueInd = True

                # Read competitor's static indication
                # staticIndication = isStatic[n>>3] & (1<<(n & 0x0007))
                shift = n>>3
                n16 = int2ba(n,length=16, signed=False)
                barr7 = int2ba(0x0007,length=16, signed=False)
                shiftOne = ba2int(n16 & barr7)
                one16 = int2ba(1,length=16, signed=False)
                mask = one16 << shiftOne
                mask = mask[8:16]

                staticIndication = (isStatic[shift:shift+8] & mask)

                if (score < bestScore[n]):
                    # Competition won
                    # Check whether unique indicator needs to be cleared
                    # Unique indicator is not cleared when dynamic point beats a static point or a ghost point
                    if((unitInstance['isTargetStatic'] == False) and         # Dynamic target is a winner
                        ((staticIndication == True) or                       # Dynamic target wins against static one
                         (bestIndex[n] == GTRACK_ID_GHOST_POINT_LIKELY) or   # Dynamic target wins agains likely ghost
                         (bestIndex[n] == GTRACK_ID_GHOST_POINT_BEHIND))):   # Dynamic target wins agains further away ghost

                        clearUniqueInd = False
                        isStatic[shift:shift+8] &= ~mask

                    # Register score and the index
                    bestScore[n] = score
                    bestIndex[n] = tid
                    point[n][3] = rvOut # doppler

                else:
                    # Competition lost
                    # Check whether unique indicator needs to be cleared
                    # Unique indicator is not cleared in the following case:
                    if((staticIndication == False) and              # Loss to dynamic point
                        ((unitInstance['isTargetStatic']) or        # Static loses to dynamic
                         (tid == GTRACK_ID_GHOST_POINT_BEHIND) or   # Weak ghost looses to dynamic
                         (isInsideInnerGate == False))):            # Point outsude inner gate loses to dynamic

                        clearUniqueInd = False

                if clearUniqueInd:
                    isUnique[shift:shift+8] &= ~mask


    unitInstance['ec'] = unitInstance['gC_inv']


def gtrack_unitEvent(unitInstance, num, numReliable):
    """Run unit level state machine.

    Args:
        unitInstance (dict): Instance of gtrack unit
        num (int): number of associated measurements
        numReliable (int): Number of reliable (dynamic AND unique) measurements
    """

    isInsideBoundary = False
    isInsideStatic = False

    if unitInstance['params']['transformParams']['transformationRequired']:
        posW = gtrack_censor2world(unitInstance['S_hat'][:3], unitInstance['params']['transformParams'])

    else:
        posW = unitInstance['S_hat'][:3] # [posX, posY, posZ]


    for numBoxes in range(unitInstance['params']['sceneryParams']['numBoundaryBoxes']):
        if gtrack_isPointInsideBox(posW, unitInstance['params']['sceneryParams']['boundaryBox'][numBoxes]):
            isInsideBoundary = True
            break

    
    if isInsideBoundary:
        unitInstance['outside2freeCount'] = 0

    else:
        unitInstance['outside2freeCount'] +=1
        if (unitInstance['outside2freeCount'] >= unitInstance['params']['stateParams']['exit2freeThre']):
            unitInstance['state'] = TRACK_STATE_FREE

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}] OUTSIDE => FREE\n"
            log.debug(log_string)


    if (unitInstance['state'] == TRACK_STATE_DETECTION):
        if (numReliable > 3):
            # Hit event
            unitInstance['detect2freeCount'] = 0
            unitInstance['detect2activeCount'] +=1

            if (unitInstance['detect2activeCount'] > unitInstance['params']['stateParams']['det2actThre']):
                unitInstance['state'] = TRACK_STATE_ACTIVE

                log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}] DET => ACTIVE\n"
                log.debug(log_string)


        else:
            if (numReliable == 0):
                # Miss
                unitInstance['detect2freeCount'] +=1

                if(unitInstance['detect2activeCount'] > 0):
                    unitInstance['detect2activeCount'] -=1

                if (unitInstance['detect2freeCount'] > unitInstance['params']['stateParams']['det2freeThre']):
                    unitInstance['state'] = TRACK_STATE_FREE

                log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}] DET ({unitInstance['detect2freeCount']} > {unitInstance['params']['stateParams']['det2freeThre']}) => FREE\n"
                log.debug(log_string)


            else:
                unitInstance['detect2freeCount'] = 0

    elif (unitInstance['state'] == TRACK_STATE_ACTIVE):
        for numBoxes in range(unitInstance['params']['sceneryParams']['numStaticBoxes']):
            if gtrack_isPointInsideBox(posW, unitInstance['params']['sceneryParams']['staticBox'][numBoxes]):
                isInsideStatic = True
                break


        stop = False

        if (unitInstance['params']['stateParams']['sleep2freeThre'] != 0):
            if unitInstance['isTargetStatic']:
                unitInstance['sleep2freeCount'] += 1

                if isInsideStatic:
                    thre = unitInstance['params']['stateParams']['sleep2freeThre']

                else:
                    thre = unitInstance['params']['stateParams']['exit2freeThre']

                
                if (unitInstance['confidenceLevel'] < unitInstance['confidenceLevelThreshold']):
                    thre = unitInstance['params']['stateParams']['exit2freeThre']

                
                if (unitInstance['sleep2freeCount'] > thre):
                    unitInstance['state'] = TRACK_STATE_FREE

                    log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}] ACTIVE (SLEEP {unitInstance['sleep2freeCount']} > {thre}) => FREE\n"
                    log.debug(log_string)


                    # Set state to FREE then
                    # stop and exit this function
                    stop = True

            else:
                unitInstance['sleep2freeCount'] = 0

        # If state remains set to ACTIVE, continue with evaluation of Hit or Miss event.
        # Otherwise stop and exit this function
        if (not stop and num):
            # Hit event
            unitInstance['active2freeCount'] = 0

        elif not stop:
            # Miss
            unitInstance['active2freeCount'] +=1

            # Set threshold based on whether target is static, or is in exit zone
            if isInsideStatic:
                if unitInstance['isTargetStatic']:
                    # If target is static and inside static box
                    thre = unitInstance['params']['stateParams']['static2freeThre']

                else:
                    # Normal moving target
                    thre = unitInstance['params']['stateParams']['active2freeThre']

            
            else:
                # Exit zone threshold
                thre = unitInstance['params']['stateParams']['exit2freeThre']

            
            if (unitInstance['active2freeCount'] > thre):
                unitInstance['state'] = TRACK_STATE_FREE

                log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}] ACTIVE ({unitInstance['active2freeCount']} > {thre}) => FREE\n"
                log.debug(log_string)


def gtrack_unitStart(unitInstance, timeStamp, tid, unitCenter, isBehind):
    """Start target tracking. This function is called during modules' allocation step,
    once a new set of points passes allocation thresholds.

    Args:
        uid (dict): Unit instance
        timeStamp (float): Allocation timestamp
        tid (int): Target identifier given to a unit
        unitCenter (list): spherical coordinates of a centroid [range, azimuth, elevation, doppler] in units [m, rad, rad, m/s]
        isBehind (bool): Indicates whether the new unit behind another existing target or not.
    """


    unitInstance['tid'] = tid
    unitInstance['heartBeatCount'] = timeStamp*1000
    unitInstance['allocationTime'] = timeStamp*1000     # allocation time [ms]
    unitInstance['allocationRange'] = unitCenter[0]     # range [m]
    unitInstance['allocationVelocity'] = unitCenter[3]  # Doppler [m/s]

    # State and counters init
    unitInstance['state'] = TRACK_STATE_DETECTION
    unitInstance['active2freeCount'] = 0
    unitInstance['sleep2freeCount'] = 0
    unitInstance['detect2activeCount'] = 0
    unitInstance['detect2freeCount'] = 0
    unitInstance['currentStateVectorType'] = unitInstance['stateVectorType']

    unitInstance['isTargetStatic'] = False

    if isBehind:
        unitInstance['confidenceLevel'] = 0.5

    else:
        unitInstance['confidenceLevel'] = 1.0

    # Radial Velocity init
    # Radial Velocity handling is set to start with range rate filtering
    unitInstance['velocityHandling'] = VELOCITY_INIT

    # Measurement vector
    u = unitCenter

    # doppler
    u[3] = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], unitInstance['initialRadialVelocity'], unitCenter[3])
    unitInstance['rangeRate'] = u[3]

    # Initialise a-priori State information and compute cartesia velocity
    cartesian = gtrack_spherical2cartesian(unitInstance['currentStateVectorType'], u)
    unitInstance['S_apriori_hat'] = cartesian
    unitInstance['H_s'] = u # np.array(u) # Initialize Hs to measurement vector

    # Initialize measurements error limits
    measErrLimits = gtrack_calcMeasurementSpread(u[0], unitInstance['params']['gatingParams']['limits'])
    unitInstance['H_limits'] = measErrLimits

    # Initialize measurement spread estimation
    estSpread = gtrack_calcMeasurementLimits(u[0], unitInstance['params']['gatingParams']['limits'])
    unitInstance['estSpread'] = estSpread

    
    # Initialize a-priori Process covariance
    if unitInstance['currentStateVectorType'] == STATE_VECTORS_2DV or unitInstance['currentStateVectorType'] == STATE_VECTORS_2DA:
        pInit = np.array([0.0, 0.0, 0.5, 0.5, 1.0, 1.0], dtype=float) # Output matrix diagonal
        unitInstance['P_apriori_hat'] = np.diag(pInit)

        unitInstance['gD'] = np.zeros((3,3), dtype=float)
        

    elif unitInstance['currentStateVectorType'] == STATE_VECTORS_3DV or  unitInstance['currentStateVectorType'] == STATE_VECTORS_3DA:
        # unitInstance['P_apriori_hat'] = np.eye(9); pInit is used instead of eye matrix as per gtrack algorithm in 
        pInit = np.array([0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0], dtype=float) # Output matrix diagonal
        unitInstance['P_apriori_hat'] = np.diag(pInit)

        unitInstance['gD'] = np.zeros((4,4), dtype=float)


    unitInstance['G'] = unitInstance['params']['gatingParams']['gain']

    
    log_string = f'{unitInstance["heartBeatCount"]}: {unitInstance["uid"]} ALLOCATED, TID {tid}, Range {u[0]}, Azimuth {u[1]}, Elevation {u[2]}, Doppler {unitCenter[3]}=>{u[3]}\n'
    log.debug(log_string)


def gtrack_unitUpdate(unitInstance, point, pInd, isUnique, num, var = None):
    """Perform unit update step.

    Args:
        unitInstance (dict): Unit instance.
        point (2D list): Set of measurement points.
        pInd (list): List of associated UIDs.
        isUnique (binaryArray): Binary array indicating wwhether point belongs to a single atrget (1) or not (0).
        num (int): Number of measurement points.
        var (2D list, optional): Set of input measurement variances. Defaults to None if variances are unknown.

    Returns:
        [type]: [description]
    """

    if (unitInstance['currentStateVectorType'] == STATE_VECTORS_3DV or 
        unitInstance['currentStateVectorType'] == STATE_VECTORS_3DA):
        spreadMin = np.array([1.0, 10*math.pi/180.0, 0.5])

    elif (unitInstance['currentStateVectorType'] == STATE_VECTORS_3DV or 
        unitInstance['currentStateVectorType'] == STATE_VECTORS_3DA):
        spreadMin = np.array([1.0, 10*math.pi/180.0, 10*math.pi/180.0, 0.5])

    spreadMin = SPREAD_MIN


    myPointNum = 0
    myGoodPointNum = 0      # "Good" points = dynamic AND unique points
    myDynamicPointNum = 0
    myStaticPointNum = 0
    rvPilot = None

    goodPointsSNR = 0.0
    cartVelocity = 0.0

    # Initialize matrices and vectors
    matD = np.zeros((4, 4), dtype=float)        # Dispersion matrix
    matRm = np.zeros((4, 4), dtype=float)       # Measurement error covariance matrix
    matRc = np.zeros((4, 4), dtype=float)       # Measurment error covariance matrix for the centroid used for Kalman update

    u_goodPointsSum = np.zeros(4, dtype=float)
    uvar_sum = np.zeros(4, dtype=float)
    u_max = (-FLT_MAX) * np.ones(4, dtype=float)
    u_min = FLT_MAX * np.ones(4, dtype=float)
    uvar_mean = np.zeros(4, dtype=float)


    # Compute means of associated measurement points
    # Accumulate measurements
    for n in range(num):
        if(pInd[n] == unitInstance['uid']):

            u = point[n][:4]

            if (var is not None):
                uvar = var[n]
                uvar_sum += uvar

            if (math.fabs(u[3]) > FLT_EPSILON):
                # This is a dynamic point
                myDynamicPointNum +=1
                
                # Check whether the point is unique
                shift = n>>3
                n16 = int2ba(n,length=16, signed=False)
                barr7 = int2ba(0x0007,length=16, signed=False)
                shiftOne = ba2int(n16 & barr7)
                one16 = int2ba(0x1,length=8, signed=False)
                mask = one16 << shiftOne
                checkUnique = isUnique[shift:shift+8] & mask
                intIsUnique = ba2int(checkUnique)

                if intIsUnique:
                    # Yes, the point is "good" (dynamic and unique)
                    # Unroll Doppler using a pilot
                    if (myGoodPointNum == 0):
                        rvPilot = u[3]

                    else:
                        u[3] = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], rvPilot, u[3])

                    # Compute maximums and minimums for each measurement
                    for m in range(len(u)):
                        if (u[m] > u_max[m]):
                            u_max[m] = u[m]
                        
                        if (u[m] < u_min[m]):
                            u_min[m] = u[m]

                    
                    myGoodPointNum +=1

                    if unitInstance['isSnrWeighting']:
                        goodPointsSNR += point[n][4]
                        u_goodPointsSum += u * point[n][4]

                    else:
                        u_goodPointsSum += u

                    
                    # Mark points as reliable
                    pInd[n] = GTRACK_ID_RESERVED_GOOD_POINT

            else:
                myStaticPointNum +=1
            
            myPointNum +=1

    
    if unitInstance['isEnablePointNumberEstimation']:
        if myGoodPointNum:
            # Update estimated number of points
            if (myGoodPointNum > unitInstance['estNumOfPoints']):
                unitInstance['estNumOfPoints'] = float(myGoodPointNum)

            else:
                unitInstance['estNumOfPoints'] = 0.9*unitInstance['estNumOfPoints'] + 0.1*float(myGoodPointNum)


            # Lower bound for the estimated number of points is allocation threshold
            if(unitInstance['estNumOfPoints'] < unitInstance['params']['allocationParams']['pointsThre']):
                unitInstance['estNumOfPoints'] = unitInstance['params']['allocationParams']['pointsThre']

    
    if myGoodPointNum:
        # Compute measurement centroid as SNR-weighted mean of associated good points
        if unitInstance['isSnrWeighting']:
            unitInstance['uCenter'] = (1.0/goodPointsSNR)*u_goodPointsSum
        
        else:
            unitInstance['uCenter'] = (1.0/myGoodPointNum)*u_goodPointsSum

        
        unitInstance['isTargetStatic'] = False

        # Unroll centroid radial velocity based on target state
        rvIn = unitInstance['uCenter'][3]
        gtrack_velocityStateHandling(unitInstance, unitInstance['uCenter'])

        # Compute measurements centroid in cartesian space
        unitInstance['uPos'] = gtrack_sph2cart(unitInstance['uCenter'])

        # Compute mean measurment variance, if availbale
        if (var is not None):
            uvar_mean = (1.0/float(myPointNum)) * uvar_sum

    
    # Update measurement spread if we have 2 or more good points
    if (myGoodPointNum > 1):
        for m in range(len(u_max)):
            spread = u_max[m] - u_min[m]

            # Unbiased spread estimation
            spread = spread * (myGoodPointNum+1) / (myGoodPointNum-1)

            # Spread can't be larger than 2x of configured limits
            if (spread > 2*unitInstance['H_limits'][m]):
                spread = 2*unitInstance['H_limits'][m]

            # Spread can't be smaller than configured limits
            if (spread < spreadMin[m]):
                spread = spreadMin[m]

            
            if (spread > unitInstance['estSpread'][m]):
                unitInstance['estSpread'][m] = spread

            else:
                unitInstance['estSpread'][m] = (1.0 - unitInstance['estSpreadAlpha'])*unitInstance['estSpread'][m] + unitInstance['estSpreadAlpha']*spread

        unitInstance['estDim'] = gtrack_calcDim(unitInstance['estSpread'], unitInstance['uCenter'][0])

    
    if (unitInstance['isTargetStatic'] == False):
        # Handle potential transitioning from dynamic to static state
        # We can only transition to static when either there are no points or there are only static points available
        if ((myPointNum == 0) or (myDynamicPointNum == 0)):
            # Compute cartesian velocity
            for n in range(unitInstance['stateVectorDimNum']):
                if (unitInstance['isAssociationHeightIgnore'] and (n == 1)):
                    continue

                positionVel = GTRACK_VEL_DIMENSION * unitInstance['stateVectorDimNum'] +n
                vel = unitInstance['S_apriori_hat'][positionVel]
                cartVelocity += vel*vel


            cartVelocity = math.sqrt(cartVelocity)

        
        if (myPointNum == 0):
            # Erasures handling: no measurements available
            if (cartVelocity < unitInstance['minStaticVelocityOne']):
                # Force zero velocity/zero acceleration
                for n in range(unitInstance['stateVectorDimNum']):
                    positionVel = GTRACK_VEL_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    positionAcc = GTRACK_ACC_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    unitInstance['S_apriori_hat'][positionVel] = 0.0
                    unitInstance['S_apriori_hat'][positionAcc] = 0.0


                unitInstance['isTargetStatic'] = True

            else:
                # Target is moving => force constant velocity model
                # Force zero acceleration and slow down
                for n in range(unitInstance['stateVectorDimNum']):
                    positionVel = GTRACK_VEL_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    positionAcc = GTRACK_ACC_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    unitInstance['S_apriori_hat'][positionVel] *= 0.5
                    unitInstance['S_apriori_hat'][positionAcc] = 0.0

        
        elif (myDynamicPointNum == 0):
            # Handling only static points
            if (cartVelocity < unitInstance['minStaticVelocityTwo']):
                # slow enough => complete stop
                for n in range(unitInstance['stateVectorDimNum']):
                    positionVel = GTRACK_VEL_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    positionAcc = GTRACK_ACC_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    unitInstance['S_apriori_hat'][positionVel] = 0.0
                    unitInstance['S_apriori_hat'][positionAcc] = 0.0

                if (myStaticPointNum > 3):
                    # Confirmed with many static points => set it to static
                    unitInstance['isTargetStatic'] = True

            else:
                # Slow down, keep dynamic
                for n in range(unitInstance['stateVectorDimNum']):
                    positionVel = GTRACK_VEL_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    positionAcc = GTRACK_ACC_DIMENSION * unitInstance['stateVectorDimNum'] +n
                    unitInstance['S_apriori_hat'][positionVel] *= 0.5
                    unitInstance['S_apriori_hat'][positionAcc] = 0.0


    else:
        # Handle potential transitioning from static to dynamic state
        if (myDynamicPointNum > 0):
            unitInstance['isTargetStatic'] = False

    # Update target confidence level
    if myGoodPointNum:
        if unitInstance['numAssosiatedPoints']:
            confidenceUpdate = float(myGoodPointNum + (myDynamicPointNum-myGoodPointNum)/2)/unitInstance['numAssosiatedPoints']

        else:
            confidenceUpdate = 0

        
        unitInstance['confidenceLevel'] = (1.0-GTRACK_CONFIDENCE_ALPHA)*unitInstance['confidenceLevel'] + GTRACK_CONFIDENCE_ALPHA*confidenceUpdate


    if myGoodPointNum:
        # Compute matRm, Measurement Noise covariance matrix
        if var is None:
            for m, estSpread in enumerate(unitInstance['estSpread']):
                # Spread is covered by 2 sigmas
                # sigma = unitInstance['estSpread'][m]/2
                sigma = estSpread/2
                uvar_mean[m] = sigma*sigma

        
        matRm = np.diag(uvar_mean)

    
    # Update Group Dispersion gD matrix
    if myGoodPointNum:
        # matD is the new dispersion matrix
        for n in range(num):
            if (pInd[n] == GTRACK_ID_RESERVED_GOOD_POINT):
                pInd[n] = unitInstance['uid']

                # Accumulate covariance from all associated points
                gtrack_matrixCovAcc(matD, point[n], unitInstance['uCenter'])

        
        # Covariance matrix normalization
        gtrack_matrixCovNormalize(matD, myGoodPointNum)

        if (myGoodPointNum > GTRACK_MIN_POINTS_TO_UPDATE_DISPERSION):
            # Update persistant group dispersion based on instantaneous matD
            # The factor alpha goes from maximum (1.0) at the first allocation down to minimum of 0.1 once the target has been observed for a long time
            alpha = float(myGoodPointNum)/unitInstance['estNumOfPoints']

            # Filter covariance matrix with filtering coefficient alpha
            unitInstance['gD'] = (1-alpha)*unitInstance['gD'] + alpha*matD


    if myGoodPointNum:
        # Compute state vector partial derivatives (Jacobian matrix)
        matJ = gtrack_computeJacobian(unitInstance['currentStateVectorType'], unitInstance['S_apriori_hat'])
        matPJT = np.matmul(unitInstance['P_apriori_hat'], matJ.T)
        matJPJT = np.matmul(matJ, matPJT)

        # Compute centroid measurement noise covariance matrix matRc used for Kalman updates
        # First term represents the error in measuring the centroid and decreased with the number of measurments
        # Second term represents the centroid unsertanty due to the fact that not all the memebers observed
        # matRc = matRm/num + alpha*unitInstance['gD']*eye(mSize);  mSize = 3 (2D); mSize = 4 (3D)

        # alpha is weighting factor that is the function of the number of observed poionts versus total number of reflections in a group
        # alpha = (unitInstance['maxAssociatedPoints']-num)/((unitInstance['maxAssociatedPoints']-1)*num) unitInstance['maxAssociatedPoints']

        alpha = (float(unitInstance['estNumOfPoints'])-myGoodPointNum)/((unitInstance['estNumOfPoints']-1)*myGoodPointNum)

        rm_diag = np.diag(unitInstance['gD']).copy()

        for n in range(len(uvar_mean)):
            rm_diag[n] = uvar_mean[n]/myGoodPointNum + rm_diag[n]*alpha

        matRc = np.diag(rm_diag)
        

        if log.isEnabledFor(logging.DEBUG):
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: spread\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['estSpread']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Rm\n"
            log.debug(log_string)
            log_string = np.array_str(matRm).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: D\n"
            log.debug(log_string)
            log_string = np.array_str(matD).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: gD\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['gD']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Rc\n"
            log.debug(log_string)
            log_string = np.array_str(matRc).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: S-apriori\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['S_apriori_hat']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: J\n"
            log.debug(log_string)
            log_string = np.array_str(matJ).replace('[', '').replace(']', '').strip()
            log.debug(log_string)
        

        # Compute innovation
        u_tilda = unitInstance['uCenter'] - unitInstance['H_s']

        if log.isEnabledFor(logging.DEBUG):
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: u_tilda\n"
            log_string = np.array_str(u_tilda).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

        
        # Compute centroid covariance matcC
        matcC = matJPJT + matRc

        # Compute inverse of matcC
        matcC_inv = np.linalg.inv(matcC)

        if log.isEnabledFor(logging.DEBUG):
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: P\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['P_apriori_hat']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)
            
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: cC\n"
            log.debug(log_string)
            log_string = np.array_str(matcC).replace('[', '').replace(']', '').strip()
            log.debug(log_string)

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: cC_inv\n"
            log.debug(log_string)
            log_string = np.array_str(matcC_inv).replace('[', '').replace(']', '').strip()
            log.debug(log_string)



        # Compute Kalman Gain K
        # K = P_apriori_hat * matJ * matcC_inv
        matK = np.matmul(matPJT, matcC_inv)

        if log.isEnabledFor(logging.DEBUG):
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: K\n"
            log.debug(log_string)
            log_string = np.array_str(matK).replace('[', '').replace(']', '').strip()
            log.debug(log_string)



        # State estimation
        # S_hat = S_apriori_hat + K*u_tilda
        unitInstance['S_hat'] = unitInstance['S_apriori_hat'] + np.matmul(matK, u_tilda)

        # Covariance estimation
        # P_hat = P_apriori_hat - matK*matJ*matP
        unitInstance['P_hat'] = unitInstance['P_apriori_hat'] - np.matmul(matK, matPJT.T)


        # Compute groupCovariance gC (will be used in gating)
        # We will use ellipsoidal gating, that acounts for the dispersion of the group, target maneuver, and measurement noise
        unitInstance['gC'] = unitInstance['gD'] + matJPJT + matRm

        # Compute inverse of group innovation
        unitInstance['gC_inv'] = np.linalg.inv(unitInstance['gC'])

        # This method may return a negative determinant, which causes an issue when calculating a logarithm of gC_det in gtrack_unitScore() function.
        # unitInstance['gC_det'] = np.linalg.det(unitInstance['gC'])
        # For this reason, logarithm of the determinant is computed directly in using slogdet() method.
        sign, logdet = np.linalg.slogdet(unitInstance['gC'])
        unitInstance['gC_det'] = [sign * np.exp(logdet), logdet] # [determinant, logarithm of determinant]

        if log.isEnabledFor(logging.DEBUG):
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: gC\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['gC']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)
                
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: gC_inv\n"
            log.debug(log_string)
            log_string = np.array_str(unitInstance['gC_inv']).replace('[', '').replace(']', '').strip()
            log.debug(log_string)


    else:
        # Handling of erasures
        if (myDynamicPointNum == 0):
            unitInstance['skipPrediction'] = True


        unitInstance['S_hat'] = unitInstance['S_apriori_hat']
        unitInstance['uPos'] = unitInstance['S_apriori_hat'][:3]
        unitInstance['P_hat'] = unitInstance['P_apriori_hat']


    if log.isEnabledFor(logging.DEBUG):
        log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: S-hat\n"
        log.debug(log_string)
        log_string = np.array_str(unitInstance['S_hat']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)
        unitInstance['S_hat']
        
        log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: P-hat\n"
        log.debug(log_string)
        log_string = np.array_str(unitInstance['P_hat']).replace('[', '').replace(']', '').strip()
        log.debug(log_string)
        unitInstance['P_hat']


    if log.isEnabledFor(logging.DEBUG):
        log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: \n"

        if unitInstance['isTargetStatic']:
            log_string +=  f"(ST, {unitInstance['confidenceLevel']}), "

        else:
            log_string +=  f"(DYN, {unitInstance['confidenceLevel']}), "

        
        if myPointNum:
            log_string +=  f"Update, {myPointNum}({myGoodPointNum}, {myStaticPointNum}, {unitInstance['numAssosiatedPoints']}) points, "

            if (unitInstance['state'] == TRACK_STATE_DETECTION):
                log_string +=  f", DET {unitInstance['detect2activeCount']}, "

                if myGoodPointNum:
                    log_string +=  f"XYZD={{{unitInstance['uPos'][0]}, {unitInstance['uPos'][1]}, {unitInstance['uPos'][2]}, {rvIn} => {unitInstance['uCenter'][3]}}}, "

        else:
            log_string +=  'Miss, '

            if unitInstance['isTargetStatic']:
                log_string +=  'Static, '

            else:
                log_string +=  'Moving, '
            
            if (unitInstance['state'] == TRACK_STATE_DETECTION):
                log_string +=  f"DET {unitInstance['detect2activeCount']}, "

            else:
                log_string +=  f"ACT {unitInstance['active2freeCount']}, "

        
        log_string +=  'dim={'
        for m in range(len(unitInstance['estDim'])-1):
            log_string += f"{unitInstance['estDim'][m]}, "

        log_string += f"{unitInstance['estDim'][m+1]}}}, "


        log_string +=  'S={'
        for n in range(len(unitInstance['S_hat'])-1):
            log_string += f"{unitInstance['S_hat'][n]}, "

        log_string += f"{unitInstance['S_hat'][n+1]}}}\n"
        log.debug(log_string)


    gtrack_unitEvent(unitInstance, myPointNum, myGoodPointNum)

    return unitInstance['state']


def gtrack_unitReport(unitInstance, t, tNum):
    """Report gtrack unit results to the target descriptor

    Args:
        unitInstance (dict): Unit instance
        targetDesc (dict): Target descriptor
    """

    targetDesc = {}
    targetDesc['uid'] = unitInstance['uid']
    targetDesc['tid'] = unitInstance['tid']

    targetDesc['S'] = unitInstance['S_hat']
    targetDesc['EC'] = unitInstance['ec']
    targetDesc['G'] = unitInstance['G']
    targetDesc['dim'] = unitInstance['estDim']
    targetDesc['uCenter'] = unitInstance['uCenter']
    targetDesc['confidenceLevel'] = unitInstance['confidenceLevel']

    description  = f"Description of a target with uid[{targetDesc['uid']}] and tid[{targetDesc['tid']}].\n"
    description += f"Target position at x: {targetDesc['S'][0]:.2f} m, y: {targetDesc['S'][1]:.2f} m, z: {targetDesc['S'][2]:.2f} m.\n"
    description += f"Target velocity in direction x: {targetDesc['S'][3]:.2f} m/s, y: {targetDesc['S'][4]:.2f} m/s, z: {targetDesc['S'][5]:.2f} m/s.\n"
    description += f"Target acceleration in direction x: {targetDesc['S'][6]:.2f} m/s2, y: {targetDesc['S'][7]:.2f} m/s2, z: {targetDesc['S'][8]:.2f} m/s2.\n"
    description += f"Target gain factor: {targetDesc['G']}.\n"
    description += f"Estimated target dimensions, depth: {targetDesc['dim'][0]:.2f}, width: {targetDesc['dim'][1]:.2f}, height: {targetDesc['dim'][2]:.2f}.\n"
    description += f"Estimated radial velocity of target: {targetDesc['dim'][3]:.2f}.\n"
    description += f"Measurement centroid position, range: {targetDesc['uCenter'][0]:.2f}, azimuth: {targetDesc['uCenter'][1]:.2f}, elevation: {targetDesc['uCenter'][2]:.2f}, doppler: {targetDesc['uCenter'][3]:.2f}.\n"
    description += f"Target confidence level: {targetDesc['confidenceLevel']:.2f}.\n"

    log.info(description)

    t.insert(tNum, targetDesc)