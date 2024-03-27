import rosbag
import csv
import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi
# from scipy.signal import butter, sosfilt, filtfilt
from scipy import signal
from scipy.integrate import cumulative_trapezoid

# SELECT: 
# 1) Scenario:
CIRCLE = "circle"
TOWN = "town"

SCENARIO = TOWN
# 2) Convert to csv:
# Set this to false after we add corrected  magnetometer.
# So that we don't overwrite from original rosbag.
CONVERT_ROSBAG_TO_CSV = True
# 3) Compute oadev again:
COMPUTE_OADEV = False
if CONVERT_ROSBAG_TO_CSV == True:
    COMPUTE_OADEV = True


# Global constants
IMAGE_EXTENSION = "png"
TOPIC = "/imu"
FIGSIZE = (18, 12)

# Create the bag and csv filepaths.
DATA_DIR = "../data"
PLOT_DIR = f"{DATA_DIR}/plots/{SCENARIO}"

imu_filename = SCENARIO + "_imu"
gps_filename = SCENARIO + "_gps"
imu_bag_filepath = f'{DATA_DIR}/{SCENARIO}/{imu_filename}.bag'
bag_filepaths = [imu_bag_filepath]

imu_csv_filepath = f'{DATA_DIR}/{SCENARIO}/{imu_filename}.csv'
gps_csv_filepath = f'{DATA_DIR}/{SCENARIO}/{gps_filename}.csv'
csv_filepaths = [imu_csv_filepath, gps_csv_filepath]
colors = {'x': 'blue', 'y': 'green', 'z': 'red'}

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


def clean_and_convert_to_float(value_str):
    # Attempt to clean the string by removing unexpected characters
    cleaned_str = value_str.strip().replace('\x00', '')  # Remove null characters and whitespace

    # Replace multiple decimal points, if any, with a single one
    # This is a simplistic approach and might need adjustment based on actual data errors
    parts = cleaned_str.split('.')
    if len(parts) > 2:  # More than one decimal point
        cleaned_str = parts[0] + '.' + ''.join(parts[1:])

    try:
        # Attempt to convert the cleaned string to a float
        return float(cleaned_str)
    except ValueError as e:
        # Handle conversion errors
        print(f"Error converting to float: {e}. Original string: '{value_str}', Cleaned string: '{cleaned_str}'")
        return None  # Or choose a default value, or raise an exception

def convert_rosbag_to_csv(bag_filepaths, csv_filepaths):
    for i in range(len(bag_filepaths)):
        bag_filepath = bag_filepaths[i]
        csv_filepath = csv_filepaths[i]
        first_stamp = None
        # Open the rosbag
        with rosbag.Bag(bag_filepath, 'r') as bag, open(csv_filepath, 'w', newline='') as csvfile:
            fieldnames = ['seq', 'stamp',
                          'elapsed_time', # time from 0
                          'yaw', 'pitch', 'roll', 
                          'mag_x', 'mag_y', 'mag_z',
                          'accel_x', 'accel_y', 'accel_z',
                          'gyro_x', 'gyro_y', 'gyro_z', 
                          ]
            # Create the csv file
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            i = 0
            # Iterate through each row and get each field.
            for topic, msg, t in bag.read_messages(topics=[TOPIC]):
                # print("\nmsg_data=", msg.header)

                # Calculate elapsed time since the start of the bag file
                stamp = msg.header.stamp
                if first_stamp is None:
                    first_stamp = stamp.to_sec()
                    elapsed_time = 0
                else:
                    elapsed_time = stamp.to_sec() - first_stamp

                # Parse the vnymr string (not from the VectorNav object). 
                # Since its already available just from the string and its already in degree.
                vnymrSplit = msg.vnymr_read.split(',') 

                # Get the yaw pitch roll (deg)
                yaw = clean_and_convert_to_float(vnymrSplit[VNYMR.Yaw])
                pitch = clean_and_convert_to_float(vnymrSplit[VNYMR.Pitch])
                roll = clean_and_convert_to_float(vnymrSplit[VNYMR.Roll])

                # Get the mag data
                mag_x = clean_and_convert_to_float(vnymrSplit[VNYMR.MagX])
                mag_y = clean_and_convert_to_float(vnymrSplit[VNYMR.MagY])
                mag_z = clean_and_convert_to_float(vnymrSplit[VNYMR.MagZ])
                
                # Get accel x, y z (m/s^2)
                accel_x = clean_and_convert_to_float(vnymrSplit[VNYMR.AccelX])
                accel_y = clean_and_convert_to_float(vnymrSplit[VNYMR.AccelY])
                accel_z = clean_and_convert_to_float(vnymrSplit[VNYMR.AccelZ])

                # Get the gyro data (rad/s^2)
                gyro_x = clean_and_convert_to_float(vnymrSplit[VNYMR.GyroX])
                gyro_y = clean_and_convert_to_float(vnymrSplit[VNYMR.GyroY])
                gyro_z_str = vnymrSplit[VNYMR.GyroZ].split('*')[0]  # Split off any potential checksum before cleaning
                gyro_z = clean_and_convert_to_float(gyro_z_str)


                # Write to csv
                row_dict = {
                    'seq': msg.header.seq,
                    'stamp': msg.header.stamp.to_sec(),
                    'elapsed_time': elapsed_time,
                    'yaw': yaw,
                    'pitch': pitch,
                    'roll': roll,
                    'mag_x': mag_x,
                    'mag_y': mag_y,
                    'mag_z': mag_z,
                    'accel_x': accel_x,
                    'accel_y': accel_y,
                    'accel_z': accel_z,
                    'gyro_x': gyro_x,
                    'gyro_y': gyro_y,
                    'gyro_z': gyro_z,
                }
                writer.writerow(row_dict)

    print("finish convert rosbag to csv")


def plot_magnetic_components(data):
    """
    Plots the North vs. East components of the magnetic field from the dataset before calibration.
    
    Parameters:
    - data: pandas DataFrame containing the dataset with magnetic field components.
    - plot_dir: String. The directory where the plot image will be saved.
    - figsize: Tuple. The figure size.
    """
    if 'mag_x' not in data.columns or 'mag_y' not in data.columns:
        raise ValueError("DataFrame must contain 'mag_x' and 'mag_y' columns")

    # Setup for the "ideal" magnetic field model
    N = len(data)  
    
    # Extract measured magnetic field from the DataFrame
    x_meas = data['mag_x'].values
    y_meas = data['mag_y'].values
    X_meas = np.array([x_meas, y_meas])

    # Plotting
    fig, axs = plt.subplots(1, 1, figsize=FIGSIZE)
    
    # Before Calibration
    axs.scatter(X_meas[0], X_meas[1], c='blue', label='Before Calibration')
    axs.set_title('Magnetic North vs. East Components Before Calibration')
    axs.set_xlabel('East Component (mag_x)')
    axs.set_ylabel('North Component (mag_y)')
    axs.grid(True)
    axs.axis('equal')
    axs.legend()
    
    # Save the plot
    plot_path = os.path.join(PLOT_DIR, 'mag_field_before_calib.png')
    plt.savefig(plot_path)



if __name__ == '__main__':
    if not os.path.exists(PLOT_DIR):
        os.makedirs(PLOT_DIR)

    # Convert bag file to csv.
    if CONVERT_ROSBAG_TO_CSV:
        convert_rosbag_to_csv(bag_filepaths, csv_filepaths)

    # Load data from csv and plot
    imu_data = pd.read_csv(csv_filepaths[0])
    gps_data = pd.read_csv(csv_filepaths[1])

    # plt.figure(figsize=(20,16))
    # plt.plot(imu_data['stamp'].values, imu_data['yaw'].values, label='IMU yaw', color='blue', linewidth=1)

    # plt.title('Gyro and Magnetometer Heading: High-pass vs Low-pass vs Complementary Filter')
    # plt.xlabel('Time Stamp')
    # plt.ylabel('Heading (degrees)')
    # plt.legend()
    # plt.grid(True)
    # plt.show()





