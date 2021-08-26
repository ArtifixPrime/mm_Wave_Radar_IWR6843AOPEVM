import math
import numpy as np
from .constants import FLT_MAX, FLT_MIN


# Constants for 3D space
GTRACK_NOMINAL_RANGE_SPREAD                         = 0.5               # Default value to initialize the range spread
GTRACK_NOMINAL_ANGULAR_SPREAD                       = 2*math.pi/180     # Default value to initialize the angular spread
GTRACK_NOMINAL_DOPPLER_SPREAD                       = 1.0               # Default value to initialize the doppler spread

GTRACK_BORESIGHT_STATIC_ANGLE                       = 6*math.pi/180
GTRACK_BORESIGHT_STATIC_RANGE                       = 2.0

SPREAD_MIN = np.array([1.0, 10*math.pi/180.0, 10*math.pi/180.0, 0.5])


def gtrack_sph2cart(sph):
    """Convert a measurement point vector from spherical to cartesian.

    Args:
        sph (list): Measurements vector in spherical form [range, azimuth, elevation]

    Returns:
        list: State vector in cartesian form [x, y, z]
    """

    c = np.zeros(3, dtype=float)
    azim = sph[1]
    elev = sph[2]

    c[0] = sph[0] * math.cos(elev) * math.sin(azim) # posX
    c[1] = sph[0] * math.cos(elev) * math.cos(azim) # posY
    c[2] = sph[0] * math.sin(elev)                  # posZ

    return  c


def gtrack_cart2sph(cart):
    """Convert a vector from cartesian to spherical.

    Args:
        cart (list): State vector in cartesian form [x, y, z]

    Returns:
        list: Measurements vector in spherical form [range, azimuth, elevation]
    """

    sph = np.zeros(3, dtype=float)

    sph[0] = math.sqrt(cart[0]*cart[0] + cart[1]*cart[1] + cart[2]*cart[2])     # range
    sph[1] = math.atan(cart[0]/cart[1])                                         # azimuth
    sph[1] = math.atan(cart[2] / math.sqrt(cart[0]*cart[0] + cart[1]*cart[1]))  # elevation

    return sph


def gtrack_censor2world(cart_in, world):
    """Transform cartesian coordinates from sensor-centric space to world (global) space.

    Args:
        cart_in (list): [posX, posY, posZ]
        world (dict): World transformation parameters.

    Returns:
        list: Cartesian coordinates in world space.
    """

    cart_out = np.zeros(3, dtype=float)

    # Rotation around X axis (elevation tilt)
    # xOut = xIn
    cart_out[0] = cart_in[0]
    # yOut = yIn*cos(theta) + zIn*sin(theta)
    cart_out[1] = cart_in[1]*world['rotX'][1] + cart_in[2]*world['rotX'][0]

    # zOut = -yIn*sin(theta) + zIn*cos(theta) + offsetZ
    cart_out[2] = -cart_in[1]*world['rotX'][0] + cart_in[2]*world['rotX'][1] + world['offsetZ']


    return cart_out


def gtrack_calcMeasurementSpread(rnge, gate_limits):
    """Compute measurement spread based on current target position and configred target dimensions.

    Args:
        rnge (float): Target range.
        gate_limits (dict): Target dimensions limits.

    Returns:
        list: Measurement spread.
    """

    estSpread = np.zeros(4, dtype=float) # [range, azimuth, elevation, doppler]

    # Range spread
    if (gate_limits['depth'] <= FLT_MIN):
        estSpread[0] = 0.5 # Default value to initialize the range spread

    else: 
        estSpread[0] = gate_limits['depth']


    # Azimuth spread
    if (gate_limits['width'] <= FLT_MIN):
        estSpread[1] = 2*math.pi/180.0 # Default value to initialize the angular spread

    else: 
        estSpread[1] = 2*math.atan((gate_limits['width']/2)/rnge)


    # Elevation spread
    if (gate_limits['height'] <= FLT_MIN):
        estSpread[2] = 2*math.pi/180.0 # Default value to initialize the angular spread

    else: 
        estSpread[2] = 2*math.atan((gate_limits['width']/2)/rnge)


    estSpread[3] = 1.0 # Initial value of doppler spread

    return estSpread


def gtrack_calcMeasurementLimits(rnge, gate_limits):
    """Compute measurement error limits based on current target position and configured target dimensions.

    Args:
        rnge (float): Target range.
        gate_limits (dict): Measurement error limits.

    Returns:
        list: Measurement error limits.
    """

    limits = np.zeros(4, dtype=float) # [range, azimuth, elevation, doppler]

    # Range limit
    if (gate_limits['depth'] <= FLT_MIN):
        limits[0] = FLT_MAX

    else: 
        limits[0] = gate_limits['depth']/2


    # Azimuth limit
    if (gate_limits['width'] <= FLT_MIN):
        limits[1] = FLT_MAX

    else: 
        limits[1] = 2*math.atan((gate_limits['width']/2)/rnge)


    # Elevation limit
    if (gate_limits['height'] <= FLT_MIN):
        limits[2] = FLT_MAX

    else: 
        limits[2] = 2*math.atan((gate_limits['height']/2)/rnge)


    # Doppler limit
    if (gate_limits['vel'] <= FLT_MIN):
        limits[3] = FLT_MAX

    else: 
        limits[3] = gate_limits['vel']/2


    return limits


def gtrack_calcDim(mSpread, rnge):
    """Compute target dimension estimations based on estimated measurement spread and centroid range.

    Args:
        mSpread (list): Measurement spread vector.
        rnge (float): Scalar range.

    Returns:
        list: Estimated target dimensions.
    """

    tDim = np.zeros(4, dtype=float)
    tDim[0] = mSpread[0]
    tDim[1] = 2*rnge*math.tan(mSpread[1]/2)
    tDim[2] = 2*rnge*math.tan(mSpread[2]/2)
    tDim[3] = mSpread[3]

    return tDim


def gtrack_calcDistance(sph1, sph2):
    """Calculate a distance between two points defined in spherical coordinates. Angles are in radians.

    Args:
        sph1 (list): [range1, azimuth1, elevation1]
        sph2 (list): [range2, azimuth2, elevation2]

    Returns:
        float: Square of a distance in m^2.
    """

    azimDiff = sph1[1] - sph2[1]
    cosAzim = math.cos(azimDiff)

    sinElev1 = math.sin(sph1[2])
    sinElev2 = math.sin(sph2[2])

    cosElev1 = math.cos(sph1[2])
    cosElev2 = math.cos(sph2[2])

    distance = sph1[0]*sph1[0] + sph2[0]*sph2[0] - 2*sph1[0]*sph2[0]*(cosElev1*cosElev2*cosAzim + sinElev1*sinElev2)

    return distance


def gtrack_computeMahalanobis(vector, matD):
    """Compute Mahanalobis distance between vector and distribution matrix D.

    Args:
        vector (list): Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
        matrix (2D list): Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.

    Returns:
        float: Mahalanobis distance in form.
               md = v*D*v'
    """
    # Using only NumPY
    md = np.sqrt(np.sum(np.dot(vector, matD) * vector))


    # Another method using SciPy (needs to be imported first at the top)
    # SciPy calculates Mahalanobis distance between two 1-D arrays.
    # Since input vector already represents the delta of two 1-D array, a zero vector is substracted from it.
    # zero = np.zeros(len(vector))
    # md = distance.mahalanobis(vector, zero, matD)

    return md


def gtrack_computeMahalanobisPartial(vector, matD):
    """Compute partial Mahalanobis distance between vector and distributin matrix D.

    Args:
        vector (list): Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
 		               The last dimension of vector is ignored (vector[3] = 0).
        matD (2D list): Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.

    Returns:
        float: Partial Mahalanobis distance.
    """

    # Partial Mahalanobis distance does not take doppler component into account.
    matTmp = np.delete(matD[:3], np.s_[3], axis=1) # Matrix without last row and column of matD
    md_part = np.sqrt(np.sum(np.dot(vector[:3], matTmp) * vector[:3]))

    return md_part


def gtrack_isPointBehindTarget(point, unitCentroid, spread):
    """Check whether measurement point is geometrically behind the target (unit).

    Args:
        point (list): [range, azimuth, elevation, doppler]
        unitCentroid (list): target centroid in spherical coordinates.
        spread (list): estimated spread of target centroid in spherical coordinates.

    Returns:
        bool: True if point is behind target, False otherwise.
    """

    azimuthCond = math.fabs(point[1] - unitCentroid[1]) < spread[1]/2
    elevCond = math.fabs(point[2] - unitCentroid[2]) < spread[2]/2
    dopplerCond = math.fabs(point[3] - unitCentroid[3]) < 2*spread[3]
    rangeCond = point[0] > unitCentroid[0]

    if (azimuthCond and elevCond and dopplerCond and rangeCond):
        return True

    else:
        return False


def gtrack_isInsideSolidAngle(measError, measSpread):
    """Check whether angular error is less than angular spread.

    Args:
        measError (list): Measurement vector (spherical).
        measSpread (list): Measurement vector (spherical), representing target spread.

    Returns:
        bool: True if target is inside, False otherwise.
    """

    azimuthError = math.fabs(measError[1])
    elevationError = math.fabs(measError[2])

    if(azimuthError < measSpread[1]/2) and (elevationError < measSpread[2]/2):

        return True

    else:

        return False


def gtrack_isPointInsideBox(point, box):
    """Check whether the point is inside the box boundaries or not.

    Args:
        point (list): point in cartesian space [x, y, z]
        box (dict): 3D box boundaries.

    Returns:
        bool: True if inside, False otherwise.
    """

    xBound = (point[0] > box['x1']) and (point[0] < box['x2'])
    yBound = (point[1] > box['y1']) and (point[1] < box['y2'])
    zBound = (point[2] > box['z1']) and (point[2] < box['z2'])

    if(xBound and yBound and zBound):
        return True

    else:
        return False


def gtrack_isInsideBoresightStaticZone(v):
    """Check whether measurement point is within boresight static zone.

    Args:
        v (list): Measurements vector in spherical form of [range, azimuth, elevation]

    Returns:
        bool: True if inside, False otherwise.
    """

    if (
        (v[0] > GTRACK_BORESIGHT_STATIC_RANGE) and              # range
        (math.fabs(v[1]) < GTRACK_BORESIGHT_STATIC_ANGLE) and   # azimuth
        (math.fabs(v[2]) < GTRACK_BORESIGHT_STATIC_ANGLE)       # elevation
    ): 
        return True

    else: 
        return False
