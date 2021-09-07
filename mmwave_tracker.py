import math
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import logging
import json

logging.basicConfig(level=logging.INFO)
log = logging.getLogger('tracker.log')
log.debug('tracker.log initialized')

import gtrack
from gtrack import tracking_algorithm

# Change the configuration file name
configFileName = 'config_file-midRange-5m_noGrouping.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;


#
MAX_TRACKS = gtrack.constants.GTRACK_NUM_TRACKS_MAX
MAX_POINTS = gtrack.constants.GTRACK_NUM_POINTS_MAX
STATE_VECTOR = gtrack.constants.STATE_VECTORS_3DA
BENCHMARK_SIZE = gtrack.constants.GTRACK_BENCHMARK_SIZE





# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyUSB0', 115200)
    #Dataport = serial.Serial('/dev/ttyUSB1', 921600)
    
    # Windows
    CLIport = serial.Serial('COM3', 115200)
    Dataport = serial.Serial('COM4', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);  # frame rate in Hz

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["samplesPerChirp"] = numAdcSamples
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    configParameters["frameRate"] = framePeriodicity
    
    return configParameters
   
# ------------------------------------------------------------------

# Function to set configuration for object tracking
def trackingConfig(configParameters):

    # GTRACK advanced config parameters
    appSceneryParams = {
        'sensorPosition': np.array([0.0, 0.0, 0.85], dtype=float),
        'sensorOrientation': np.array([0.0, 0.0], dtype=float),
        'numBoundaryBoxes': 1, # One boundary box
        'boundaryBox':[
            {
                'x1': -1.5, # left boundary in meters
                'x2':  1.5, # right boundary in meters
                'y1':  0.1, # near boundary in meters
                'y2':  5.0, # far boundary in meters
                'z1':  0.0, # bottom boundary in meters
                'z2':  3.0  # top boundary in meters
            },
            {
                'x1':  0.0,
                'x2':  0.0,
                'y1':  0.0,
                'y2':  0.0,
                'z1':  0.0,
                'z2':  0.0
            }
        ],
        'numStaticBoxes': 1, # One static box
        'staticBox': [
            {
                'x1': -1.0,
                'x2':  1.0,
                'y1':  0.1,
                'y2':  1.0,
                'z1':  0.0,
                'z2':  2.0
            },
            {
                'x1':  0.0,
                'x2':  0.0,
                'y1':  0.0,
                'y2':  0.0,
                'z1':  0.0,
                'z2':  0.0
            }
        ]
    }

    boundaries = {}

    boundaries['boundaryBox'] = appSceneryParams['boundaryBox'][0]
    boundaries['staticBox'] = appSceneryParams['staticBox'][0]


    appGatingParams = {
        'gain': 3.0,
        'limits': {
            'depth':  1.5,
            'width':  1.5,
            'height': 2.0,
            'vel':    1.0
        }
    }

    appStateParams = {
        'det2actThre':      10,
        'det2freeThre':     50,
        'active2freeThre':  50,
        'static2freeThre':  100,
        'exit2freeThre':    50,
        'sleep2freeThre':   1000
    }

    appAllocationParams = {
        'snrThre': 36.0,
        'snrThreObscured': 200.0,
        'velocityThre': 0.1,
        'pointsThre': 9,
        'maxDistanceThre': 1.1,
        'maxVelThre': 3.0
    }

    appPresenceDetectionParams = {
        'pointsThre': 5,                        # occupancy threshold, number of points. Setting pointsThre to 0 disables presence detection
        'velocityThre': 0.0,                    # occupancy threshold, approaching velocity
        'on2offThre': 0,                        # occupancy on to off threshold
        'numOccupancyBoxes': 1,                 # Number of occupancy boxes. Presence detection algorithm will determine whether the combined shape is occupied. Setting numOccupancyBoxes to 0 disables presence detection.
        'occupancyBox': [                       # Scene occupancy boxes.
            {
                'x1': -1.0,
                'x2':  1.0,
                'y1':  0.2,
                'y2':  1.5,
                'z1':  0.0,
                'z2':  3.0
            },
            {
                'x1':  0.0,
                'x2':  0.0,
                'y1':  0.0,
                'y2':  0.0,
                'z1':  0.0,
                'z2':  0.0
            }
        ]
    }


    # GTRACK config
    configGtrack = {
        'stateVectorType': STATE_VECTOR,
        'verbose': 3,                                                           # Level DEBUG - currently this setting DOES NOT DO ANYTHING
        'deltaT': configParameters["frameRate"]*0.001,                          # Frame rate in ms
        'maxRadialVelocity': configParameters["maxVelocity"],                   # Radial velocity from sensor is limited to +/- maxURV (in m/s)
        'radialVelocityResolution': configParameters["dopplerResolutionMps"],   # Radial velocity resolution (in m/s)
        'maxAcceleration': [0.0, 4.0, 0.0],                                     # Maximum expected target acceleration in lateral (X), longitudinal (Y), and vertical (Z) directions, m/s2.
        'maxNumPoints': 250,
        'maxNumTracks': 20,
        'initialRadialVelocity': 1,                                             # Expected target radial velocity at the moment of detection, m/s
        'advParams': {
            'gatingParams': appGatingParams,
            'stateParams': appStateParams,
            'allocationParams': appAllocationParams,
            'sceneryParams': appSceneryParams,
            'presenceParams': appPresenceDetectionParams
        }
    }


    return configGtrack, boundaries

# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData68xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    FFT_SIZE = configParameters["samplesPerChirp"];

    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    MMWDEMO_UART_MSG_NOISE_PROFILE   = 3;
    MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP = 4;
    MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP = 5;
    MMWDEMO_UART_MSG_STATS = 6;
    MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO = 7;
    MMWDEMO_UART_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP = 8;
    MMWDEMO_UART_MSG_TEMPERATURE_STATS = 9;
    MMWDEMO_UART_MSG_MAX = 10;

    maxBufferSize = 2**15;
    tlvHeaderLengthInBytes = 8;
    pointLengthInBytes = 16;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    PI = 3.14159265
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    detObj_sideInfo = {}
    rangeProfile = []
    noiseProfile = []
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        # mmwave_sdk_03_05_00_04/packages/ti/demo/xwr64xx/mmw/docs/doxygen/html/struct_mmw_demo__output__message__header__t.html
        magicNumber = byteBuffer[idX:idX+8]
        assert np.array_equal(magicNumber, magicWord)

        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        assert version == '3050004'

        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        assert numTLVs <= 10

        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4

        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4

            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                # Initialize the arrays
                # mmwave_sdk_03_05_00_04/packages/ti/datapath/dpu/rangeproc/docs/doxygen/html/struct_d_p_i_f___point_cloud_cartesian__t.html
                x = np.zeros(numDetectedObj,dtype=np.float32)
                y = np.zeros(numDetectedObj,dtype=np.float32)
                z = np.zeros(numDetectedObj,dtype=np.float32)
                velocity = np.zeros(numDetectedObj,dtype=np.float32)
                # Doppler velocity estimate in m/s. 
                # Positive velocity means target is moving away from the sensor and 
                # negative velocity means target is moving towards the sensor.

                # arrays for calculated values
                objectRange = np.zeros(numDetectedObj,dtype=np.float32)
                objectAzimuth = np.zeros(numDetectedObj,dtype=np.float32)
                objectElevation = np.zeros(numDetectedObj,dtype=np.float32)
                
                for objectNum in range(numDetectedObj):
                    
                    # Read the data for each object
                    x[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    y[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    z[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velocity[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4

                    # Calculate range, azimuth and elevation
                    # range of detected object
                    objectRange[objectNum] = float(np.sqrt(
                        float(x[objectNum]) * float(x[objectNum]) + 
                        float(y[objectNum]) * float(y[objectNum]) + 
                        float(z[objectNum]) * float(z[objectNum])
                    ))

                    # azimuth of detected object
                    if y[objectNum] == np.float32(0.0): # y can never be negative as that would mean detected object is behind the sensor
                        if x[objectNum] >= np.float32(0.0):
                            objectAzimuth[objectNum] = float(90)
                        else:
                            objectAzimuth[objectNum] = float(-90)
                    else:
                        objectAzimuth[objectNum] = float(math.atan(float(x[objectNum]) / float(y[objectNum])) * (180.0 / PI))

                    # elevation of detected object
                    rngeXY = math.sqrt((float(x[objectNum]) * float(x[objectNum]))+(float(y[objectNum]) * float(y[objectNum])))
                    if (x[objectNum] == np.float32(0.0)) and (y[objectNum] == np.float32(0.0)) or (rngeXY == 0.0):
                        if z[objectNum] >= np.float32(0.0):
                            objectElevation[objectNum] = float(90)
                        else: 
                            objectElevation[objectNum] = float(-90)
                    else:
                        objectElevation[objectNum] = float(math.atan(float(z[objectNum]) / rngeXY) * (180.0 / PI))

                    
                
                # Store the data in the detObj dictionary
                detObj = {
                    "numObj": numDetectedObj, 
                    "x": x, "y": y, "z": z, 
                    "range": objectRange, "azimuth": objectAzimuth, "elevation": objectElevation, 
                    "velocity":velocity}


            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO:

                # Initialize the arrays
                # mmwave_sdk_03_05_00_04/packages/ti/datapath/dpu/rangeproc/docs/doxygen/html/struct_d_p_i_f___point_cloud_side_info__t.html
                # Point cloud side information such as SNR and noise level.
                # The structure describes the field for a point cloud in XYZ format
                
                snr = np.zeros(numDetectedObj,dtype=np.int16)
                # snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB

                noise = np.zeros(numDetectedObj,dtype=np.int16)
                # y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB

                for objectNum in range(numDetectedObj):

                    # Read the data for each object
                    snr[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.int16) * 0.1 # dB step
                    idX += 2
                    noise[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.int16) * 0.1 # dB step
                    idX += 2

                # Store side information of detected objects
                detObj_sideInfo = {
                    "snr": snr, "noise": noise
                }

                dataOK = 1

            # If range profile is enabled in guiMonitor, TLV type 2 package will be added to the UART output packet
            if tlv_type == MMWDEMO_UART_MSG_RANGE_PROFILE:
                continue

                # Initialize the array for raw data
                power = np.zeros(FFT_SIZE,dtype=np.uint16)

                for sample in range(FFT_SIZE):

                    power[sample] = byteBuffer[idX:idX + 2].view(dtype=np.uint16)

                    # Convert uint16 from Q7.9 format (Q9 for short) to floating point
                    # First 7 bits in uint16 represent the whole (integer) part of a single sample
                    integer_bits = '{0:b}'.format(power[sample])[:7]
                    integer = int(integer_bits, 2)

                    fraction_bits = '{0:b}'.format(power[sample])[7:]
                    fraction = 0.0

                    for bit in range(len(fraction_bits)):
                        multiplier = int(fraction_bits[bit])
                        exponent = -bit - 1
                        fraction += multiplier * pow(2, exponent)

                    rangeProfile.append(integer + fraction) # in dB
                    idX += 2

            # If noise profile is enabled in guiMonitor, TLV type 3 package will be added to the UART output packet 
            if tlv_type == MMWDEMO_UART_MSG_NOISE_PROFILE:
                continue

                # Initialize the array for raw data
                noise = np.zeros(FFT_SIZE,dtype=np.uint16)

                for sample in range(FFT_SIZE):

                    noise[sample] = byteBuffer[idX:idX + 2].view(dtype=np.uint16)

                    # Convert uint16 from Q7.9 format to floating point
                    integer_bits = '{0:b}'.format(noise[sample])[:7]
                    integer = int(integer_bits, 2)

                    fraction_bits = '{0:b}'.format(noise[sample])[7:]
                    fraction = 0.0

                    for bit in range(len(fraction_bits)):
                        multiplier = int(fraction_bits[bit])
                        exponent = -bit - 1
                        fraction += multiplier * pow(2, exponent)

                    noiseProfile.append(integer + fraction) # in dB
                    idX += 2




        # Remove already processed data
        if idX > 0 and byteBufferLength>idX:
            shiftSize = totalPacketLen
            
                
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0         

    return dataOK, frameNumber, detObj, detObj_sideInfo, rangeProfile, noiseProfile

# ------------------------------------------------------------------

# Function to perform a single stracking step
def updateTracking(frameNum, mNum, trackModule, pointCloud, targetDescriptor, presence, benchmarks):

    gTick = frameNum

    # Limit the points
    if mNum > MAX_POINTS:
        mNum = MAX_POINTS
    
    startTime = time.monotonic()
    tNum, presence = tracking_algorithm.gtrack_step(trackModule, pointCloud, mNum, targetDescriptor, presence, bench=None)
            
    benchmarkTime = time.monotonic() - startTime
    


# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update(pointCloud, ax):
     
    dataOk = 0
    global detObj
    #global detObj_sideInfo
    x = []
    y = []
    #cloud, = ax.scatter(x, y)

    # Read and parse the received data
    dataOk, frameNumber, detObj, detObj_sideInfo, rangeProfile, noiseProfile = readAndParseData68xx(Dataport, configParameters)
    mNum = 0
    goodPoints = 0
    totalSnr = 0.0
    cloud_v = []
    
    if dataOk and len(detObj["x"])>0:
        mNum = detObj['numObj']
        #print(detObj)
        #print(detObj_sideInfo)
        #print(rangeProfile)
        #print(noiseProfile)
        #x = -detObj["x"]
        #y = detObj["y"]

        x = detObj['x']
        #print('------------- X -------------')
        #print(x)
        y = detObj['y']
        
        #print('------------- Y -------------')
        #print(y)
        #ax.xlim(x_boundaries)
        #ax.ylim(y_boundaries)
        
        #cloud.set_xdata(x)
        #cloud.set_ydata(y)
        #figure.canvas.draw()
        #figure.canvas.flush_events()


        cloud = ax.scatter(x, y)
        #figure.canvas.draw()
        #figure.canvas.flush_events()
        #cloud.set_xdata(x)
        plt.pause(0.01)
        #time.sleep(0.01)
        cloud.remove()
        
        for n in range(mNum):
            # detPoint = [detObj['range'], detObj['azimuth'], detObj['elevation'], detObj['velocity'], detObj_sideInfo['snr']]
            pointCloud.append(
                np.array(
                    [detObj['range'][n],
                    math.radians(detObj['azimuth'][n]),
                    math.radians(detObj['elevation'][n]),
                    detObj['velocity'][n],
                    detObj_sideInfo['snr'][n]],
                    dtype=float
                )
            )

            #if (detObj['x'][n] >= -0.75) and (detObj['x'][n] <= 0.75):
            #    if (detObj['y'][n] >= 1.00) and (detObj['y'][n] <= 2.5):
            #        goodPoints += 1
            #        totalSnr += detObj_sideInfo['snr'][n]
            #        cloud_v.append(detObj['velocity'][n])

    #if len(cloud_v) > 0:
    #    max_v = max(cloud_v)
    #    min_v = min(cloud_v)
    #    delta_v = math.fabs(max_v - min_v)

    #else:
    #    delta_v = 0.0

            

        """json_list.append({
                'range': float(detObj['range'][n]),
                'azimuth': math.radians(detObj['azimuth'][n]),
                'elevation': math.radians(detObj['elevation'][n]),
                'velocity': float(detObj['velocity'][n]),
                'snr': float(detObj_sideInfo['snr'][n])
            })"""



    #if totalSnr > 0.0:
    #    with open('static-1.csv', 'a') as fw:
    #        fw.write(f"{goodPoints}, {totalSnr:.2f},{delta_v:.2f}\n")
        """with open('pointCloud.json', 'a') as fw:
            fw.write(f'{json.dumps(json_list)},')"""


    

    return dataOk, frameNumber, mNum


# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)
configGtrack, boundaries = trackingConfig(configParameters)

x_boundaries = (boundaries['boundaryBox']['x1'], boundaries['boundaryBox']['x2'])
x_static = (boundaries['staticBox']['x1'], boundaries['staticBox']['x2'])

y_boundaries = (boundaries['boundaryBox']['y1'], boundaries['boundaryBox']['y2'])
y_static = (boundaries['staticBox']['y1'], boundaries['staticBox']['y2'])

z_boundaries = (boundaries['boundaryBox']['z1'], boundaries['boundaryBox']['z2'])
z_static = (boundaries['staticBox']['z1'], boundaries['staticBox']['z2'])


plt.style.use('ggplot')
plt.ion()

#figure = plt.figure()
#ax = figure.add_subplot(projection='3d')
figure, ax = plt.subplots(figsize=(8,6))


ax.set_xlim(x_boundaries)
ax.set_ylim(y_boundaries)

#plt.show()


pointCloud = []
targetDescriptor = []
benchmarks = [0]*BENCHMARK_SIZE
benchPerTrackTotal = 0
benchPerTrackCount = 0
benchPerTrackTotalAve = 0
benchPerTrackTotalMax = 0
benchPerTrackTotalMin  = 4294967295

presence = 1


trackingModules = {}
tracking_algorithm.gtrack_create(configGtrack, 'person', trackingModules)

startTimeGtrack = time.monotonic()

# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0
while True:
    try:
        tCheckNum = 0

        # Update the data and check if the data is okay

        dataOk, frameNumber, mNum = update(pointCloud, ax)
        plt.show()

        tCheckNum
        
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = detObj
            currentIndex += 1
            #print(pointCloud)


            #updateTracking(frameNumber, mNum, trackingModules['person'], pointCloud, targetDescriptor, presence, benchmarks)
            pointCloud.clear()

        
        #time.sleep(0.1) # Sampling frequency of 20 Hz
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        print('Sensor stopped.')
        CLIport.close()
        Dataport.close()
        #win.close()
        break
        
    