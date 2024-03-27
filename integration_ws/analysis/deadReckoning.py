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
CONVERT_ROSBAG_TO_CSV = False
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

# Low-pass filter design function
def butter_lowpass_filter(raw_data, cutoff_freq, sampl_freq, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = signal.butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype='lowpass', analog=False, output='sos')
    filtered_data = signal.sosfilt(sos, raw_data)
    return filtered_data

# High-pass filter design function
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

# High-pass filter application function
def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = signal.filtfilt(b, a, data)
    return y


def apply_complementary_filter(data, alpha):
    """
    Apply a complementary filter to combine the high-pass filtered gyro data
    and low-pass filtered magnetometer data.

    Args:
    data: The DataFrame with your heading data.
    alpha: The filter constant used for blending the two signals. It's a value
           between 0 and 1, where 0 means only the magnetometer data is used,
           and 1 means only the gyro data is used.

    Returns:
    Updated DataFrame with the complementary filtered heading.
    """
    # alpha is typically close to 1. You will have to tune it based on your application's needs.
    # For example, alpha might be 0.98, which means the filter trusts the gyro 98% and the magnetometer 2%.
    
    # The complementary filter equation is:
    # heading_complementary = alpha * heading_gyro + (1 - alpha) * heading_magnet
    data['heading_complementary'] = alpha * data['heading_gyro_high_filtered'] + (1 - alpha) * data['heading_magnet_low_filtered']
    
    # You can also wrap the result between 0 and 360 degrees if needed
    data['heading_complementary'] = np.mod(data['heading_complementary'], 360)

    return data

# Define the filter application function
def plot_filtered_heading(imu_data):
    # Low-pass filter requirements
    low_order = 2
    low_cutoff = 0.09  # desired cutoff frequency of the filter, Hz
    low_fs = 40

    # Apply the low-pass filter to the magnetometer imu_data
    imu_data['heading_magnet_low_filtered'] = butter_lowpass_filter(imu_data['heading_magnet'], low_cutoff, low_fs, low_order)

    # Plotting for verification
    # plt.figure(figsize=(15, 7))
    # plt.plot(imu_data['stamp'].values, imu_data['heading_magnet'].values, label='Gyro Heading', color='blue', linewidth=1)
    # plt.plot(imu_data['stamp'].values, imu_data['heading_magnet_low_filtered'].values, label='Low-pass Magnet Heading', color='red', linewidth=1)
    
    # plt.title('Magnet Heading: Unfiltered vs Filtered')
    # plt.xlabel('Time Stamp')
    # plt.ylabel('Heading (degrees)')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    # High-pass filter requirements
    high_order = 2
    high_cutoff = 0.00001  # desired cutoff frequency of the filter, Hz
    high_fs = 40

    # Apply the high-pass filter to the magnetometer imu_data
    imu_data['heading_gyro_high_filtered'] = butter_highpass_filter(imu_data['heading_gyro'], high_cutoff, high_fs, high_order)

    # Plotting for verification
    # plt.figure(figsize=(15, 7))
    # plt.plot(imu_data['stamp'].values, imu_data['heading_gyro'].values, label='Gyro Heading', color='blue', linewidth=1)
    # plt.plot(imu_data['stamp'].values, imu_data['heading_gyro_high_filtered'].values, label='High-pass Filtered Gyro Heading', color='red', linewidth=1)
    
    # plt.title('Gyro Heading: Unfiltered vs Filtered')
    # plt.xlabel('Time Stamp')
    # plt.ylabel('Heading (degrees)')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    # Save the DataFrame including the new filtered columns to a CSV file
    imu_data.to_csv(imu_csv_filepath, index=False)

    # Use the function on your imu_data
    alpha = 0.5  # Example value, adjust based on your system and tests
    imu_data = apply_complementary_filter(imu_data, alpha)

    # Get the IMU imu_data.
    # Unwrap the complementary and IMU yaw imu_data.
    imu_data['heading_complementary'] = np.unwrap(np.radians(imu_data['heading_complementary']))
    imu_data['yaw_unwrapped'] = np.unwrap(np.radians(imu_data['yaw']))

    # Convert back to degrees for plotting.
    imu_data['heading_complementary'] *= np.degrees(1)
    imu_data['yaw_unwrapped'] *= np.degrees(1)

    # Plot the results
    plt.figure(figsize=(20,16))
    plt.plot(imu_data['stamp'].values, imu_data['heading_gyro_high_filtered'].values, label='High-pass Filtered Gyro Heading', color='red', linewidth=1)
    plt.plot(imu_data['stamp'].values, imu_data['heading_magnet_low_filtered'].values, label='Low-pass Filtered Magnet Heading', color='green', linewidth=1)
    plt.plot(imu_data['stamp'].values, imu_data['heading_complementary'].values, label='Complementary Filtered Heading', color='purple', linewidth=1)
    plt.plot(imu_data['stamp'].values, imu_data['yaw_unwrapped'].values, label='IMU yaw', color='blue', linewidth=1)

    plt.title('Gyro and Magnetometer Heading: High-pass vs Low-pass vs Complementary Filter')
    plt.xlabel('Time Stamp')
    plt.ylabel('Heading (degrees)')
    plt.legend()
    plt.grid(True)
    
    plot_path = os.path.join(PLOT_DIR, f'plot_3_filtered_heading.png')
    plt.savefig(plot_path)



# Question: How to correct velocity? 
# First correct accel data?
def correct_drift(imu_data, stationary_period=(0, 10), fs=40):
    """
    Corrects the drift in acceleration imu_data by applying a high pass filter and
    zero-offset correction based on a stationary period.

    Args:
        imu_data (DataFrame): The imu_data containing the acceleration and timestamps.
        stationary_period (tuple): The start and end time in seconds where the device is stationary.
        fs (float): Sampling frequency in Hz.

    Returns:
        DataFrame: The corrected DataFrame.
    """
    corrected_data = imu_data.copy()
    
    for axis in ['x', 'y', 'z']:
        accel_data =imu_data[f'accel_{axis}'].to_numpy()
        
        # Zero-offset correction based on stationary period
        offset = np.mean(accel_data[stationary_period[0]*fs:stationary_period[1]*fs])
        corrected_data[f'accel_{axis}_corrected'] = accel_data - offset
        
        # Apply high-pass filter
        corrected_data[f'accel_{axis}_corrected'] = butter_highpass_filter(
            corrected_data[f'accel_{axis}_corrected'], 
            0.000001,  # Cutoff frequency
            fs, 
            1, # order
        )

    return corrected_data


def integrate_accel_twice(imu_data):
    """
    Integrate acceleration imu_data to get velocity and displacement using cumulative_trapezoid.
    """
    time = imu_data['elapsed_time'].to_numpy()
    dt = np.diff(time)  # Time intervals between measurements

    # Initialize dictionaries to store velocity and displacement imu_data
    velocity = {'x': np.zeros_like(time), 'y': np.zeros_like(time), 'z': np.zeros_like(time)}
    displacement = {'x': np.zeros_like(time), 'y': np.zeros_like(time), 'z': np.zeros_like(time)}

    for axis in ['x', 'y', 'z']:
        accel_data = imu_data[f'accel_{axis}'].to_numpy()

        # Integrate acceleration to get velocity
        velocity[axis] = cumulative_trapezoid(accel_data, time, initial=0)

        # Integrate velocity to get displacement
        displacement[axis] = cumulative_trapezoid(velocity[axis], time, initial=0)

    return velocity, displacement

def plot_accel_velocity_displacement_imu(imu_data):
    """
    Plot Acceleration, Velocity, and Displacement against time for each axis in different figures.
    """
    imu_data = correct_drift(imu_data)

    velocity, displacement = integrate_accel_twice(imu_data)
    time = imu_data['elapsed_time'].to_numpy()

    
    # Plotting imu_data
    for quantity, quantity_data in zip(['Acceleration', 'Velocity', 'Displacement'], [imu_data, velocity, displacement]):
        fig = plt.figure(figsize=FIGSIZE)
        fig.patch.set_facecolor('#f0f0f0')  # Set the face color for the figure here

        for i, axis in enumerate(['x', 'y', 'z'], 1):
            if quantity == 'Acceleration':
                y_data = imu_data[f'accel_{axis}'].dropna().to_numpy()
            else:
                y_data = quantity_data[axis]
            plt.subplot(3, 1, i)
            plt.plot(time, y_data, label=f'{quantity} {axis.upper()}', color=colors[axis])
            plt.xlabel('Time (s)')
            plt.ylabel(f'{quantity} ({"m/s^2" if quantity == "Acceleration" else "m/s" if quantity == "Velocity" else "m"})')
            plt.title(f'{quantity} {axis.upper()}')
            plt.legend(loc='upper right')
            plt.grid(True, color='white')
        
        plt.subplots_adjust(hspace=0.5)
        plot_path = os.path.join(PLOT_DIR, f'imu_{quantity.lower()}.png')
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


    plot_filtered_heading(imu_data)

    plot_accel_velocity_displacement_imu(imu_data)




