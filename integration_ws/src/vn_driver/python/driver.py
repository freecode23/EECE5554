#!/usr/bin/env python3
import rospy
import serial
import os
import math
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header


# from <package name>.<folder name> import filename
from vn_driver.msg import Vectornav

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


def compute_checksum(command):
    # Exclude the initial '$' and the '*' with the checksum placeholder
    checksum = 0
    for char in command[1:]:
        checksum ^= ord(char)
    # Convert the checksum to a two-character hexadecimal string
    hex_checksum = format(checksum, '02X')
    return hex_checksum

class VNYMR:
    Yaw = 1
    Pitch = 2
    Roll = 3
    MagX = 4
    MagY = 5
    MagZ = 6
    AccelX = 7
    AccelY = 8
    AccelZ = 9
    GyroX = 10
    GyroY = 11
    GyroZ = 12

def convert_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    # Convert degrees to radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    qx = math.sin(roll_rad/2) * math.cos(pitch_rad/2) * math.cos(yaw_rad/2) - math.cos(roll_rad/2) * math.sin(pitch_rad/2) * math.sin(yaw_rad/2)
    qy = math.cos(roll_rad/2) * math.sin(pitch_rad/2) * math.cos(yaw_rad/2) + math.sin(roll_rad/2) * math.cos(pitch_rad/2) * math.sin(yaw_rad/2)
    qz = math.cos(roll_rad/2) * math.cos(pitch_rad/2) * math.sin(yaw_rad/2) - math.sin(roll_rad/2) * math.sin(pitch_rad/2) * math.cos(yaw_rad/2)
    qw = math.cos(roll_rad/2) * math.cos(pitch_rad/2) * math.cos(yaw_rad/2) + math.sin(roll_rad/2) * math.sin(pitch_rad/2) * math.sin(yaw_rad/2)
    # Return the quaternion
    return qx, qy, qz, qw
    
def isVNYMRinString(inputString):
    if '$VNYMR' in inputString:
        return True
    else:
        return False

def parseVNYMR(vnymrStr: str, vectorNavMsg: Vectornav) -> Vectornav:
    '''
    This function parses the vnymrStr input to Vectornav ROS message and return the message object.
    $VNYMR,-165.970,-037.299,+001.252,+00.2894,+00.0706,+00.7482,-05.961,-00.184,-07.853,+00.000885,-00.000192,-00.000642*69
    '''
    # Split into components
    vnymrSplit = vnymrStr.split(',')  # Split the string by comma
    vectorNavMsg.vnymr_read = vnymrStr

    # Parse each component and convert to the appropriate data type
    yaw = float(vnymrSplit[VNYMR.Yaw]) # in degrees.
    pitch = float(vnymrSplit[VNYMR.Pitch])
    roll = float(vnymrSplit[VNYMR.Roll])
    mag_x = float(vnymrSplit[VNYMR.MagX])
    mag_y = float(vnymrSplit[VNYMR.MagY])
    mag_z = float(vnymrSplit[VNYMR.MagZ])
    accel_x = float(vnymrSplit[VNYMR.AccelX])
    accel_y = float(vnymrSplit[VNYMR.AccelY])
    accel_z = float(vnymrSplit[VNYMR.AccelZ])
    gyro_x = float(vnymrSplit[VNYMR.GyroX])
    gyro_y = float(vnymrSplit[VNYMR.GyroY])
    gyro_z = float(vnymrSplit[VNYMR.GyroZ].split('*')[0])

    # Assign all fields to the message object
    # 1. IMU
    # - orientation from the quaternion (radians)
    quaternion = convert_to_quaternion(roll, pitch, yaw)
    vectorNavMsg.imu.orientation = Quaternion(*quaternion)
    # print(f"\nroll={roll}, pitch={pitch}, yaw={yaw}")
    # print("quarternion=", quaternion)

    # - angular velocity (rad/s)
    vectorNavMsg.imu.angular_velocity = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)

    # - linear acceleration (m/s^2)
    vectorNavMsg.imu.linear_acceleration = Vector3(x=accel_x, y=accel_y, z=accel_z)

    # 2. Mag Field
    vectorNavMsg.mag_field.magnetic_field = Vector3(x=mag_x, y=mag_y, z=mag_z)

    return vectorNavMsg


if __name__ == '__main__':
    baud_rate = 115200
    vn_rate = 40
    node_name = "vn_driver_node"
    topic_name = "/imu"
    frame_id = "imu1_frame"

    # 0. init ROS node and publisher.
    rospy.init_node(node_name)
    vnPub = rospy.Publisher(topic_name, Vectornav, queue_size=5)
    r = rospy.Rate(100)

    # 1. Set to the emulator or GPS puck port
    serialPortAddr = rospy.get_param('~port', '/dev/pts/3')
    serialPort = serial.Serial(serialPortAddr, baudrate=baud_rate, timeout=5)

    # 2. Set the vectornav rate to 40 Hz
    # Convert the command to bytes, assuming the device expects UTF-8 encoding
    command = f"$VNWRG,07,{vn_rate}"  # Simplified; no checksum calculation

    # Calculate the checksum
    checksum = compute_checksum(command)
    command = f"{command}*{checksum}\r\n"
    command_bytes = command.encode('utf-8')

    # Write the command to the device
    serialPort.write(command_bytes)

    # Wait for and read the response
    response = serialPort.readline().decode()

    # 3. Prepare the directory to save the real result of vnymr string from IMU sensor.
    txtFilename = rospy.get_param('~filename', 'imu_capture')
    txtFilepath = createOutputFilepath(txtFilename)
    
    # Check if the file exists and delete it if it does
    if os.path.isfile(txtFilepath):
        os.remove(txtFilepath)

    # 4. Initialize the ROS message object.
    vectorNavMsg = Vectornav()
    vectorNavMsg.header.frame_id = frame_id
    vectorNavMsg.header.seq = 0
    vectorNavMsg.imu = Imu()
    vectorNavMsg.mag_field= MagneticField()

    # 5. Publish.
    try:
        with open(txtFilepath, 'a') as file:  # Open file in append mode
            while not rospy.is_shutdown():
                # 5.1 Read the message from port
                vnymrStr = serialPort.readline().decode('ascii').strip()
                print("vnymrStr", vnymrStr)
                if not isVNYMRinString(vnymrStr):
                    continue

                #5.2 Write vnymrStr to .txt file.
                # print("vntmrStr=", vnymrStr)
                file.write(vnymrStr + '\n')
                file.flush()
                
                # 5.3 Parse the string as vectorNav msg object and publish.
                vectorNavMsg.header.seq += 1
                vectorNavMsg.header.stamp = rospy.Time.now()
                result = parseVNYMR(vnymrStr, vectorNavMsg)
                if not result:
                    continue

                # 5.4 Publish
                vectorNavMsg = result
                vnPub.publish(vectorNavMsg)
                r.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Exception occured:", e)

    finally:
        serialPort.close()
