import numpy as np
# from scipy.spatial import distance
import math
import logging

from .constants import *


log = logging.getLogger(__name__)

def gtrack_cartesian2spherical(stateVectorType, cart):
    """Convert a vector from cartesian to spherical

    Args:
        stateVectorType (int): State vector
        cart (list): Vector describing a point in cartesian coordinates.

    Returns:
        list: Vector describing a point in spherical coordinates.
    """

    if stateVectorType == STATE_VECTORS_2DV or stateVectorType == STATE_VECTORS_2DA:
        sph = np.zeros(3, dtype=float)

        posx = cart[0]
        posy = cart[1]
        velx = cart[2]
        vely = cart[3]

        # calc range
        sph[0] = math.sqrt(posx*posx + posy*posy)

        # calc azimuth
        if posy == 0:
            sph[1] = math.pi/2

        elif posy > 0:
            sph[1] = math.atan(posx/posy)

        else:
            sph[1] = math.atan(posx/posy) + math.pi

        
        #calc Doppler
        sph[2] = (posx*velx+posy*vely)/sph[0]
        

    elif stateVectorType == STATE_VECTORS_3DV or stateVectorType == STATE_VECTORS_3DA:
        sph = np.zeros(4, dtype=float)

        posx = cart[0]
        posy = cart[1]
        posz = cart[2]
        velx = cart[3]
        vely = cart[4]
        velz = cart[5]

        # calc range
        sph[0] = math.sqrt(posx*posx + posy*posy + posz*posz)

        # calc azimuth
        if posy == 0:
            sph[1] = math.pi/2

        elif posy > 0:
            sph[1] = math.atan(posx/posy)

        else:
            sph[1] = math.atan(posx/posy) + math.pi

        # calc elevation
        sph[2] = math.atan(posz/math.sqrt(posx*posx+posy*posy))

        # calc Doppler
        sph[3] = (posx*velx+posy*vely+posz*velz)/sph[0]
    
    return sph


def gtrack_spherical2cartesian(stateVectorType, sph):
    """Convert a vector from spherical to cartesian.

    Args:
        stateVectorType (int): State vector
        sph (list): Vector describing a point in spherical coordinates.

    Returns:
        list: Vector describing a point in cartesian coordinates.
    """

    if stateVectorType == STATE_VECTORS_2DA:
        cart = np.zeros(6, dtype=float) # Ax and Ay are initialised to 0
        rnge = sph[0]
        angle = sph[1]
        doppler = sph [2]
        sinAngle = math.sin(angle)
        cosAngle = math.cos(angle)

        cart[0] = rnge*sinAngle         # X
        cart[1] = rnge*cosAngle         # Y
        cart[2] = doppler*sinAngle      # Vx
        cart[3] = doppler*cosAngle      # Vy


    elif stateVectorType == STATE_VECTORS_2DV:
        cart = np.zeros(4, dtype=float)
        rnge = sph[0]
        angle = sph[1]
        doppler = sph [2]
        sinAngle = math.sin(angle)
        cosAngle = math.cos(angle)

        cart[0] = rnge*sinAngle
        cart[1] = rnge*cosAngle
        cart[2] = doppler*sinAngle
        cart[3] = doppler*cosAngle
        

    elif stateVectorType == STATE_VECTORS_3DA:
        cart = np.zeros(9, dtype=float)     # Last three elements are initialized to 0 (acceleration in x, y, z)
        rnge = sph[0]
        azim = sph[1]
        elev = sph [2]
        doppler = sph[3]

        sinAzim = math.sin(azim)
        cosAzim = math.cos(azim)
        sinElev = math.sin(elev)
        cosElev = math.cos(elev)

        cart[0] = rnge*cosElev*sinAzim      # X
        cart[1] = rnge*cosElev*cosAzim      # Y
        cart[2] = rnge*sinElev              # Z

        cart[3] = doppler*cosElev*sinAzim   # Vx
        cart[4] = doppler*cosElev*cosAzim   # Vy
        cart[5] = doppler*sinElev           # Vz

    elif stateVectorType == STATE_VECTORS_3DV:
        cart = np.zeros(6, dtype=float)
        rnge = sph[0]
        azim = sph[1]
        elev = sph [2]
        doppler = sph[3]

        sinAzim = math.sin(azim)
        cosAzim = math.cos(azim)
        sinElev = math.sin(elev)
        cosElev = math.cos(elev)

        cart[0] = rnge*cosElev*sinAzim
        cart[1] = rnge*cosElev*cosAzim
        cart[2] = rnge*sinElev

        cart[3] = doppler*cosElev*sinAzim
        cart[4] = doppler*cosElev*cosAzim
        cart[5] = doppler*sinElev
    
    return cart


def gtrack_unrollRadialVelocity(rvMax, rvExp, rvIn):
    """Unroll radial velocity from +/- rvMax form based on expected velocity value.

    Args:
        rvMax (float): Unambiguous radial velocity.
        rvExp (float): Expected radial velocity value.
        rvIn (float): Measured radial velocity value.

    Returns:
        float: Unrolled radial velocity value.
    """

    distance = rvExp - rvIn

    if (distance >= 0):
        # Going right
        factor = int((distance + rvMax)/(2*rvMax))
        rvOut = rvIn + 2*rvMax*factor

    else:
        # Going left
        factor = int((rvMax - distance)/(2*rvMax))
        rvOut = rvIn - 2*rvMax*factor

    return rvOut


def gtrack_matrixCovAcc(mtrx, vct, vct_mean):
    """Accumulate covariance matrix with variances from input vector and vector of means.

    Args:
        mtrx (2D list): Square matrix representing covariance matrix.
        vct (list): Input vector.
        vct_mean (list): Vector mean.
    """

    for row in range(len(mtrx)):
        diff1 = vct[row]-vct_mean[row]

        for col in range(len(mtrx)):
            if(col >= row):
                diff2 = vct[col]-vct_mean[col]
                mtrx[row][col] += math.fabs(diff1*diff2)


def gtrack_matrixCovNormalize(mtrx, num):
    """Normalize covariance matrix to the number of measurements and make it symmetrical across diagonal.

    Args:
        mtrx (2D list): Upper triangular square matrix representing covariance matrix.
        num (int): Number of measurements.
    """

    for row in range(len(mtrx)):
        mtrx[row][row] /= num

        for col in range(len(mtrx)):
            if(col > row):
                
                mtrx[row][col] /= num

    # Makes upper triangular square matrix symmetrical, by mirroring off diagonal elements in lower triangular part
    # without changing the diagonal.
    np.maximum(mtrx, mtrx.T)


def gtrack_computeJacobian(stateVectorType, cart):
    """Compute partial derivatives of the cartesian vector

    Args:
        stateVectorType (int): State vector type.
        cart (list): State vector in cartesian form.

    Returns:
        2D list: Jacobian matrix of partial derivatives.
    """

    if ((stateVectorType == STATE_VECTORS_2DV) or (stateVectorType == STATE_VECTORS_2DA)):
        # STATE_VECTORS_2DV: cart = [posx, posy, velx, vely]
        #                    jacobian is 3x4
        # STATE_VECTORS_2DA: cart = [posx, posy, velx, vely, accx, accy]
        #                    jacobian is 3x6
        if (stateVectorType == STATE_VECTORS_2DV):
            jac = np.zeros((3,4), dtype=float)
        elif (stateVectorType == STATE_VECTORS_2DA):
            jac = np.zeros((3,6), dtype=float)

        posX = cart[0]
        posY = cart[1]
        velx = cart[2]
        vely = cart[3]

        rnge2 = posX*posX + posY*posY
        rnge = math.sqrt(rnge2)
        rnge3 = rnge*rnge2

        # dR
        jac[0][0] = posX/rnge   # dx
        jac[0][1] = posY/rnge   # dy

        # dPhi (dAzimuth)
        jac[1][0] = posY/rnge2  # dx
        jac[1][1] = -posX/rnge2 # dy

        # dR'
        jac[2][0] = (posY*(velx*posY - vely*posX))/rnge3    # dx
        jac[2][1] = (posX*(vely*posX - velx*posY))/rnge3    # dy
        jac[2][2] = posX/rnge                               # dx'
        jac[2][3] = posY/rnge                               # dy'


    elif ((stateVectorType == STATE_VECTORS_3DV) or (stateVectorType == STATE_VECTORS_3DA)):
        # STATE_VECTORS_3DV: cart = [posx, posy, posz, velx, vely, velz]
        #                           jacobian is 4x6
        # STATE_VECTORS_3DA: cart = [posx, posy, posz, velx, vely, velz, accx, accy, accz]
        #                           jacobian is 4x9
        if (stateVectorType == STATE_VECTORS_3DV):
            jac = np.zeros((4,6), dtype=float)
        elif (stateVectorType == STATE_VECTORS_3DA):
            jac = np.zeros((4,9), dtype=float)

        posX = cart[0]
        posY = cart[1]
        posZ = cart[2]

        velx = cart[3]
        vely = cart[4]
        velz = cart[5]

        rnge2 = posX*posX + posY*posY + posZ*posZ
        rnge = math.sqrt(rnge2)

        rngeXY2 = posX*posX + posY*posY
        rngeXY = math.sqrt(rngeXY2)

        # dR
        jac[0][0] = posX/rnge       # dx
        jac[0][1] = posY/rnge       # dy
        jac[0][2] = posZ/rnge       # dz

        # dAzim
        jac[1][0] = posY/rngeXY2    # dx
        jac[1][1] = -posX/rngeXY2   # dy
        jac[1][2] = 0               # dz

        # dElev
        jac[2][0] = -posX*posZ/(rnge2*rngeXY)   # dx
        jac[2][1] = -posY*posZ/(rnge2*rngeXY)   # dy
        jac[2][2] = rngeXY/rnge2                # dz

        # dR'
        jac[3][0] = (posY*(velx*posY - vely*posX)+posZ*(velx*posZ-velz*posX))/(rnge2*rnge)  # dx'
        jac[3][1] = (posX*(vely*posX - velx*posY)+posZ*(vely*posZ-velz*posY))/(rnge2*rnge)  # dy'
        jac[3][2] = (posX*(velz*posX - velx*posZ)+posY*(velz*posY-vely*posZ))/(rnge2*rnge)  # dy'

        jac[3][3] = posX/rnge   # dx
        jac[3][4] = posY/rnge   # dy
        jac[3][5] = posZ/rnge   # dz

    
    return jac


def gtrack_matrixMakeSymmetrical(matA):
    """Force matrix symmetry by averaging off-diagonal elements.

    Args:
        matA (2D list): Square matrix.

    Returns:
        2D list: Symmetrical square matrix.
    """

    dim = len(matA)

    matB = np.diag(np.diag(matA))

    # Keep the diagonal intact
    for row in range(dim):
        for col in range(dim):
            if row != col:
                matB[row][col] = matB[col][row] = 0.5*(matA[row][col] + matA[col][row])

    # Shorter syntax method, but it first changes the diagonal values by multiplaying them with 2 and later dividing them by 2.
    # Possibly slower.
    # matB = 0.5*(matA + matA.T)
    
    return matB


def gtrack_velocityStateHandling(unitInstance, uVec):

    rvIn = uVec[3]

    if (unitInstance['velocityHandling'] == VELOCITY_INIT):
        uVec[3] = unitInstance['rangeRate']
        unitInstance['velocityHandling'] = VELOCITY_RATE_FILTER

        log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Update vState VINIT=>VFILT, {rvIn}=>{uVec[3]}\n"
        log.debug(log_string)


    elif (unitInstance['velocityHandling'] == VELOCITY_RATE_FILTER):
        # In this state we are using filtered Rate Range to unroll radial velocity, stabilizing Range rate
        instanteneousRangeRate = (uVec[0] - unitInstance['allocationRange']) / ((unitInstance['heartBeatCount'] - unitInstance['allocationTime']) * unitInstance['dt'])

        unitInstance['rangeRate'] = unitInstance['params']['unrollingParams']['alpha'] * unitInstance['rangeRate'] + (1-unitInstance['params']['unrollingParams']['alpha']) * instanteneousRangeRate
        uVec[3] = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], unitInstance['rangeRate'], rvIn)

        rrError = (instanteneousRangeRate - unitInstance['rangeRate']) / unitInstance['rangeRate']

        if (math.fabs(rrError) < unitInstance['params']['unrollingParams']['confidence']):
            unitInstance['velocityHandling'] = VELOCITY_TRACKING

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Update vState VFILT=>VTRACK, Unrolling with RangeRate={unitInstance['rangeRate']}: {rvIn}=>{uVec[3]}\n"
            log.debug(log_string)

        else:
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Update vState VFILT, RangeRate={unitInstance['rangeRate']}, H-s={unitInstance['H_s'][3]}, rvIn={rvIn}=>{uVec[3]}\n"
            log.debug(log_string)


    elif (unitInstance['velocityHandling'] == VELOCITY_TRACKING):
        # In this state we are using filtered Rate Range to unroll radial velocity and monitoring Hs error
        instanteneousRangeRate = (uVec[0] - unitInstance['allocationRange']) / ((unitInstance['heartBeatCount'] - unitInstance['allocationTime']) * unitInstance['dt'])

        unitInstance['rangeRate'] = unitInstance['params']['unrollingParams']['alpha'] * unitInstance['rangeRate'] + (1-unitInstance['params']['unrollingParams']['alpha']) * instanteneousRangeRate
        uVec[3] = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], unitInstance['rangeRate'], rvIn)

        rvError = (unitInstance['H_s'][3] - uVec[3]) / uVec[3]
        if (math.fabs(rvError) < 0.1):
            unitInstance['velocityHandling'] = VELOCITY_LOCKED

            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Update vState VTRACK=>VLOCK, Unrolling with RangeRate={unitInstance['rangeRate']}, H-s={unitInstance['H_s'][3]}: {rvIn}=>{uVec[3]}\n"
            log.debug(log_string)
            
        else:
            log_string = f"{unitInstance['heartBeatCount']}: uid[{unitInstance['uid']}]: Update vState VTRACK=, Unrolling with RangeRate={unitInstance['rangeRate']}, H-s={unitInstance['H_s'][3]}: {rvIn}=>{uVec[3]}\n"
            log.debug(log_string)

    
    elif (unitInstance['velocityHandling'] == VELOCITY_LOCKED):
        uVec[3] = gtrack_unrollRadialVelocity(unitInstance['maxRadialVelocity'], unitInstance['H_s'][3], uVec[3])


