#!/usr/bin/env python3
import rospy
import datetime
import time
import utm
import serial

# from <package name>.<folder name> import filename
from gps_driver.msg import Customgps

def isGPGGAinString(inputString):
    if '$GPGGA' in inputString:
        return True
    else:
        return False

def degMinstoDegDec(isLongitude: bool, latOrLong: str):
    '''
    Convert latitude or longitude from degree minute into degree decimal DD.dddd format.
    '''
    degreeEndIndex = 2

    # If this is longitude.
    if isLongitude:
        degreeEndIndex += 1

    deg = float(latOrLong[:degreeEndIndex])
    mins = float(latOrLong[degreeEndIndex:]) 
    decimal_degree = deg + mins / 60.0 
    return (decimal_degree)

def latLongSignConvention(latOrLong: float, latOrLongDir: float):
    if latOrLongDir == "S" or latOrLongDir == "W":
        latOrLong = -1 * latOrLong
    return latOrLong


def convertToUTM(latitudeSigned: float, longitudeSigned: float):
    '''
    Given latitude and longitude compute all the UTM values.
    return a list of: EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER
    '''
    # The return has the form (EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER).
    UTMVals = utm.from_latlon(latitudeSigned, longitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(inputTimeStr):
    '''
    Convert input time in string format of hhmmss.ss to
    time in seconds since epoch.
    '''
    # 1. Convert input time string in hhmmss.ss to seconds as a float.
    hours = int(inputTimeStr[:2])
    minutes = int(inputTimeStr[2:4])
    seconds = int(inputTimeStr[4:6])
    fractional_seconds = float("0." + inputTimeStr[7:])
    
    # Calculate the total seconds. This is the seconds since the beginning of the day.
    bodToInputTimeSec = (hours * 3600) + (minutes * 60) + seconds + fractional_seconds
    # print("inputTimeSinceBODSec=", bodToInputTimeSec)

    # 3. Get time since epoch to the beginning of today.
    # Get the current time since epoch
    currTimeSinceEpochSec = time.time()
    # print("currTimeSinceEpochSec=", currTimeSinceEpochSec)

    # Convert it to a datetime object.
    currTimeSinceEpochDateTime = datetime.datetime.utcfromtimestamp(currTimeSinceEpochSec)

    # Reset time to midnight - beginning of today.
    bodDateTime = currTimeSinceEpochDateTime.replace(hour=0, minute=0, second=0, microsecond=0)

    # Convert beginning of today to duration in seconds since epoch.
    epochToBODsec  = time.mktime(bodDateTime.timetuple())

    # Compute the input time since epoch.
    inputTimeSinceEpochSec = epochToBODsec + bodToInputTimeSec
    # print("currentTimeSinceEpochSec", inputTimeSinceEpochSec)

    # Convert total seconds to integer and fractional part to nanoseconds
    inputTimeSec = int(inputTimeSinceEpochSec)
    inputTimeNsec = int((inputTimeSinceEpochSec - inputTimeSec) * 1e9)

    # print("inputTimeSinceEpochSec - inputTimeSec=", inputTimeSinceEpochSec - inputTimeSec)
    # print("inputTimeSec", inputTimeSec)
    # print("inputTimeNsec", inputTimeNsec)
    return [inputTimeSec, inputTimeNsec]

def parseGPGGA(gpggaStr: str, customGPSmsg: Customgps) -> Customgps:
    # Split into components
    gpggaSplit = gpggaStr.split(',')  # Split the string by comma
    UTCfloat = float(gpggaSplit[1]) #float
    latitude = gpggaSplit[2] # use string first then convert to float later
    latDir = gpggaSplit[3] #string
    longitude = gpggaSplit[4] # use string first.
    longDir = gpggaSplit[5] #string
    hdop = float(gpggaSplit[6]) #float
    altitude = float(gpggaSplit[7])

    # Convert to Signed decimal degree
    latDegDec = degMinstoDegDec(False, latitude)
    longDegDec = degMinstoDegDec(True, longitude)
    print("\ngpggaSplit=", gpggaSplit)
    print(f"latitude={latitude}, latDegDec={latDegDec}")
    print(f"longitude={longitude}, longDegDec={longDegDec}")


    latitudeSigned = latLongSignConvention(latDegDec, latDir)
    longitudeSigned = latLongSignConvention(longDegDec, longDir)
    utmVals = convertToUTM(latitudeSigned, longitudeSigned)

    # Convert to UTC epoch
    currentTime = UTCtoUTCEpoch(str(UTCfloat))

    # Assign all fields to the message object
    customGPSmsg.latitude = latitudeSigned
    customGPSmsg.longitude = longitudeSigned
    customGPSmsg.altitude = altitude
    customGPSmsg.utm_easting = utmVals[0]
    customGPSmsg.utm_northing = utmVals[1]
    customGPSmsg.zone = utmVals[2]
    customGPSmsg.letter = utmVals[3]
    customGPSmsg.hdop = hdop
    customGPSmsg.gpgga_read = gpggaStr
    customGPSmsg.header.stamp.secs = currentTime[0]
    customGPSmsg.header.stamp.nsecs = currentTime[1]

    return customGPSmsg

if __name__ == '__main__':

    # 0. init ROS node and publisher.
    rospy.init_node("gps_driver_node")
    gpsPub = rospy.Publisher("gpgga_topic", Customgps, queue_size=5)
    r = rospy.Rate(100)

    # 1. Set to the emulator or GPS puck port
    serialPortAddr = serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
    print("serialPortADdr=", serialPortAddr)
    serialPort = serial.Serial(serialPortAddr, baudrate=4800, timeout=5)

    # 2. Initialize the ROS message object.
    customGPSmsg = Customgps()
    customGPSmsg.header.frame_id = "GPS1_Frame"
    customGPSmsg.header.seq = 0

    # 3. Publish at 10Mhz.
    try:
        while not rospy.is_shutdown():
            gpggaStr = serialPort.readline().decode('ascii').strip()
            if not isGPGGAinString(gpggaStr):
                continue

            customGPSmsg.header.seq += 1
            
            # Parse the string as GPS msg object and publish.
            customGPSmsg = parseGPGGA(gpggaStr, customGPSmsg)
            gpsPub.publish(customGPSmsg)

            r.sleep()

    except rospy.ROSInterruptException:
        serialPort.close()

