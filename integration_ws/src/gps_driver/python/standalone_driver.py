#!/usr/bin/env python3
import rospy
import datetime
import utm
import serial
import os
import calendar
from decimal import Decimal


# from <package name>.<folder name> import filename
from gps_driver.msg import Customgps

class GPGGA:
    TIME = 1
    LATITUDE = 2
    LATITUDE_DIR = 3
    LONGITUDE = 4
    LONGITUDE_DIR = 5
    FIX_QUALITY = 6
    NUM_SATELLITES = 7
    HDOP = 8
    ALTITUDE = 9

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
    The EASTIN and NORTHING is distanc in meters.
    '''
    # The return has the form (EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER).
    UTMVals = utm.from_latlon(latitudeSigned, longitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(inputTimeStr: str):
    '''
    Convert input time in string format of hhmmss.ss to
    time in seconds since epoch.
    '''
    # 1. Convert input time string in hhmmss.ss to seconds as a float.
    hours = int(inputTimeStr[:2])
    minutes = int(inputTimeStr[2:4])
    seconds = int(inputTimeStr[4:6])
    fractional_seconds = Decimal("0." + inputTimeStr[7:])

    # Calculate the total seconds. This is the seconds since the beginning of the day.
    bodToInputTimeSec = (hours * 3600) + (minutes * 60) + seconds + fractional_seconds

    # 2. Get the current UTC time
    now_utc = datetime.datetime.utcnow()

    # 3. Strip the time part to get the beginning of the day (midnight)
    midnight_utc = datetime.datetime(now_utc.year, now_utc.month, now_utc.day)

    # 4. Convert back to timestamp (seconds since the Epoch) using calendar.timegm()
    epochToBODsec = calendar.timegm(midnight_utc.timetuple())

    # 5. Compute the input time since epoch.
    inputTimeSinceEpochSec = epochToBODsec + bodToInputTimeSec

    # 6. Convert total seconds to integer and fractional part to nanoseconds
    inputTimeSec = int(inputTimeSinceEpochSec)
    inputTimeNsec = int(Decimal(inputTimeSinceEpochSec - inputTimeSec) * Decimal('1e9'))

    # Modulo operation should give seconds since BOD
    print("inputTimeSec mod=", inputTimeSec%86400)

    return [inputTimeSec, inputTimeNsec]

def createOutputFilepath(filename) -> str:
    '''
    Input file name will not have file suffix.
    '''
    
    # Get the directory of the current script
    current_directory = os.path.dirname(os.path.realpath(__file__))

    # Move up three levels and then into the 'data' directory
    target_directory = os.path.join(current_directory, f'../../../data/{filename}/')

    # Create the complete filepath.
    filename += '.txt'
    txtFilepath = os.path.join(target_directory, filename)

    # Create the target directory if it doesn't exist
    if not os.path.exists(target_directory):
        os.makedirs(target_directory)

    # final filepath will be something like `catkin_ws/data/filenameA.txt`
    return txtFilepath

def parseGPGGA(gpggaStr: str, customGPSmsg: Customgps) -> Customgps:
    '''
    This function parses the gpggaStr input to Customgps ROS message and return the message object.
    '''
    # Split into components
    gpggaSplit = gpggaStr.split(',')  # Split the string by comma
    fixQuality = gpggaSplit[GPGGA.FIX_QUALITY]

    if fixQuality == "0":
        print("GPS receiver cannot provide a reliable location fix at that moment.")
        return None
    UTCstr = gpggaSplit[GPGGA.TIME] #float
    latitude = gpggaSplit[GPGGA.LATITUDE] # use string first then convert to float later
    latDir = gpggaSplit[GPGGA.LATITUDE_DIR] #string
    longitude = gpggaSplit[GPGGA.LONGITUDE] # use string first.
    longDir = gpggaSplit[GPGGA.LONGITUDE_DIR] #string
    hdop = float(gpggaSplit[GPGGA.HDOP]) #float
    altitude = float(gpggaSplit[GPGGA.ALTITUDE])

    # Convert to Signed decimal degree
    latDegDec = degMinstoDegDec(False, latitude)
    longDegDec = degMinstoDegDec(True, longitude)
    latitudeSigned = latLongSignConvention(latDegDec, latDir)
    longitudeSigned = latLongSignConvention(longDegDec, longDir)
    
    utmVals = convertToUTM(latitudeSigned, longitudeSigned)

    # Convert to UTC epoch
    currentTime = UTCtoUTCEpoch(UTCstr)

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
    customGPSmsg.header.stamp = rospy.Time.now()        

    # customGPSmsg.header.stamp.secs = currentTime[0]
    # customGPSmsg.header.stamp.nsecs = currentTime[1]
    return customGPSmsg

if __name__ == '__main__':
    # Uncomment this to parse a single line of string:
    # gpggaStr = '$GPGGA,023458.230,3401.21189,N,11824.67797,W,1,12,1.0,0.0,M,0.0,M,,*70'
    # customGPSmsg = Customgps()
    # result = parseGPGGA(gpggaStr, customGPSmsg)
    # print(result)
    
    # 0. init ROS node and publisher.
    rospy.init_node("gps_driver_node")
    gpsPub = rospy.Publisher("/gps", Customgps, queue_size=5)
    r = rospy.Rate(100)

    # 1. Set to the emulator or GPS puck port
    serialPortAddr = serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serialPort = serial.Serial(serialPortAddr, baudrate=4800, timeout=5)

    # 2. Prepare the directory to save the real result of GPGGA string from GPS puck.
    txtFilename = rospy.get_param('~filename', 'fileA')
    txtFilepath = createOutputFilepath(txtFilename)
    if os.path.isfile(txtFilepath):
        os.remove(txtFilepath)

    # 3. Initialize the ROS message object.
    customGPSmsg = Customgps()
    customGPSmsg.header.frame_id = "GPS1_Frame"
    customGPSmsg.header.seq = 0
    # 1711242770

    # 4. Publish at 10Mhz.
    try:
        with open(txtFilepath, 'a') as file:  # Open file in append mode
            while not rospy.is_shutdown():
                # 4.1 Read the message from port
                # print("test=", serialPort.readline())
                gpggaStr = serialPort.readline().decode('ascii').strip()
                if not isGPGGAinString(gpggaStr):
                    continue

                customGPSmsg.header.seq += 1

                # 4.2 Write gpggaStr to .txt file.
                print("\ngpggaStr", gpggaStr)
                file.write(gpggaStr + '\n')
                file.flush()
                
                
                # 4.3 Parse the string as GPS msg object and publish.
                result = parseGPGGA(gpggaStr, customGPSmsg)
                if not result:
                    continue

                # 4.4 Publish
                customGPSmsg = result
                gpsPub.publish(customGPSmsg)

                r.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Exception occured:", e)

    finally:
        serialPort.close()
