import rosbag
import csv
import pandas as pd
import os
import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid


# SELECT: 
# 1) Scenario:
CIRCLE_IMU = "circle_imu"
TOWN_IMU = "town_imu"

SCENARIO = CIRCLE_IMU
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

filename = SCENARIO
bag_filepath = f'{DATA_DIR}/{SCENARIO}/{filename}.bag'
bag_filepaths = [bag_filepath]

csv_filepath = f'{DATA_DIR}/{SCENARIO}/{filename}.csv'
csv_filepaths = [csv_filepath]
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


# Lab4: Dead Reckoning plots:
# 1) Fig1: Plot the N vs. E components of magnetic field and apply calibration to your dataset. 
# Plot two sub-figs before and after calibration.
# This is in MATLAB
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




# 2) Fig 2-4: Plot rotational rate and rotation (integrate once from gyro and plot magnetometer heading by calculating atan(X/Y)) vs. time. 
# Plot all three axes on three subplots per fig.
def plot_rotation_and_rotational_rate(data):
    """
    Process IMU data to plot and save:
    1) Rotational rates for GyroX, GyroY, GyroZ.
    2) Rotation calculated by integrating gyroscopic data over time.
    3) Magnetometer heading calculated from atan2(magY/magX).
    
    Parameters:
    - data: DataFrame containing time, gyroscopic data (GyroX, GyroY, GyroZ),
            and magnetometer data (magX, magY) after calibration.
    - plot_dir: Directory path where the plots will be saved.
    """
    # Plot 1: Rotational rates for GyroX, GyroY, GyroZ
    fig, axs = plt.subplots(3, 1, figsize=FIGSIZE)
    fig.patch.set_facecolor('#f0f0f0')

    for i, axis in enumerate(['x', 'y', 'z']):
        
        elapsed_time_array = data['elapsed_time'].to_numpy()  # Convert to numpy array
        gyro_data_array = data[f'gyro_{axis}'].to_numpy()  # Convert to numpy array
        axs[i].set_facecolor('#f0f0f0')
        axs[i].grid(True, color='white')
        axs[i].plot(elapsed_time_array, gyro_data_array, label=f'Gyro {axis} Rotational Rate', color=colors[axis])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Rate (rad/s)')
        axs[i].legend()

    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'rotational_rates_gyro.png'))
    plt.close(fig)

    # Plot 2: Rotation calculated by integrating gyroscopic data over time
    # Integrate gyroscopic data to get rotation for each axis
    fig, axs = plt.subplots(3, 1, figsize=FIGSIZE)
    gyro_integrated = {}
    for i, axis in enumerate(['x', 'y', 'z']):
        # Integrate gyros data to get rotation for each axis.
        gyro_integrated[axis] = cumulative_trapezoid(data[f'gyro_{axis}'].to_numpy(), 
                                                 data['elapsed_time'].to_numpy(), 
                                                 initial=0) * (180 / pi)
        axs[i].set_facecolor('#f0f0f0')
        axs[i].grid(True, color='white')
        axs[i].plot(data['elapsed_time'].to_numpy(), 
                    gyro_integrated[axis], 
                    label=f'Gyro {axis} Integrated Rotation',
                    color=colors[axis]
                    )
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Rotation (degrees)')
        axs[i].legend()
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'rotation_gyro_integrated.png'))
    plt.close(fig)

    # Plot 3: Magnetometer heading
    data['MagHeading'] = np.degrees(np.arctan2(data['mag_y_corr'], data['mag_x_corr']))
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.set_facecolor('#f0f0f0')
    ax.grid(True, color='white')

    ax.plot(data['elapsed_time'].to_numpy(), 
            data['MagHeading'].to_numpy(), 
            label='Magnetometer Heading')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Rotation (degrees)')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, 'rotation_magnetometer_heading.png'))
    plt.close(fig)


# 3) Fig 5-7: Plot acceleration, velocity (integrate once), and displacement (integrate twice) vs. time as three subplots on all three axes (Fig. 5-7)
def integrate(data):
    """
    Integrate acceleration data to get velocity and displacement using cumulative_trapezoid.
    """
    time = data['elapsed_time'].to_numpy()
    dt = np.diff(time)  # Time intervals between measurements

    # Initialize dictionaries to store velocity and displacement data
    velocity = {'x': np.zeros_like(time), 'y': np.zeros_like(time), 'z': np.zeros_like(time)}
    displacement = {'x': np.zeros_like(time), 'y': np.zeros_like(time), 'z': np.zeros_like(time)}

    for axis in ['x', 'y', 'z']:
        accel_data = data[f'accel_{axis}'].to_numpy()

        # Integrate acceleration to get velocity
        velocity[axis] = cumulative_trapezoid(accel_data, time, initial=0)

        # Integrate velocity to get displacement
        displacement[axis] = cumulative_trapezoid(velocity[axis], time, initial=0)

    return velocity, displacement

def plot_accel_velocity_displacement(data):
    """
    Plot Acceleration, Velocity, and Displacement against time for each axis in different figures.
    """
    velocity, displacement = integrate(data)
    time = data['elapsed_time'].to_numpy()
    
    # Plotting data
    for quantity, quantity_data in zip(['Acceleration', 'Velocity', 'Displacement'], [data, velocity, displacement]):
        fig = plt.figure(figsize=FIGSIZE)
        fig.patch.set_facecolor('#f0f0f0')  # Set the face color for the figure here

        for i, axis in enumerate(['x', 'y', 'z'], 1):
            if quantity == 'Acceleration':
                y_data = data[f'accel_{axis}'].dropna().to_numpy()
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
        plot_path = os.path.join(PLOT_DIR, f'{quantity.lower()}.png')
        plt.savefig(plot_path)

if __name__ == '__main__':
    if not os.path.exists(PLOT_DIR):
        os.makedirs(PLOT_DIR)

    # Convert bag file to csv.
    if CONVERT_ROSBAG_TO_CSV:
        convert_rosbag_to_csv(bag_filepaths, csv_filepaths)

    # Load data from csv and plot
    data = pd.read_csv(csv_filepaths[0])
    plot_magnetic_components(data)

    # Make sure to run the matlab manetometerCalib.py before running this.
    # run : matlab -softwareopengl
    # print("\n1. Plotting rotation and rotational rate")
    # plot_rotation_and_rotational_rate(data)

    # print("\n2. Plotting Acceleration, Velocity, Displacement")
    # plot_accel_velocity_displacement(data)







