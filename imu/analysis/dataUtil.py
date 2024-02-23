import rosbag
import csv
import pandas as pd
import plotly.graph_objects as go
import os
import numpy as np
from allantools import oadev
import matplotlib.pyplot as plt


# SELECT: 
# 1) Scenario:
LOCATION_C = "locationC"
LIVE_CAPTURE = "live_capture"
SCENARIO = LOCATION_C

# 2) Convert to csv:
CONVERT_ROSBAG_TO_CSV = False

# 3) Compute oadev again:
COMPUTE_OADEV = False
if CONVERT_ROSBAG_TO_CSV == True:
    COMPUTE_OADEV = True


# Global constants
IMAGE_EXTENSION = "png"
TAU = "Tau"
ADEV = "Adev"
TOPIC = "/imu"
if SCENARIO == LOCATION_C:
    TOPIC = "/vectornav"
FIGSIZE = (14, 12)

# Create the bag and csv filepaths.
DATA_DIR = "../data"
PLOT_DIR = f"{DATA_DIR}/plots/{SCENARIO}"

filename = SCENARIO
bag_filepath = f'{DATA_DIR}/{SCENARIO}/{filename}.bag'
bag_filepaths = [bag_filepath]

csv_filepath = f'{DATA_DIR}/{SCENARIO}/{filename}.csv'
csv_filepaths = [csv_filepath]
 
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


def plot_allan_bias_instability(taus, adevs, axis):
    # Convert to log scale
    logtau = np.log10(taus)
    logadev = np.log10(adevs)

    # Calculate slopes of the log-scaled Allan deviation
    dlogadev = np.diff(logadev) / np.diff(logtau)
    slope = 0

    # Find the index where the slope is closest to the specified slope (0)
    i = np.argmin(np.abs(dlogadev - slope))

    # Find the y-intercept of the line
    b = logadev[i] - slope * logtau[i]

    # Determine the bias instability coefficient from the line
    scfB = np.sqrt(2 * np.log(2) / np.pi)
    logB = b - np.log10(scfB)
    B = 10 ** logB

    # Plot the results
    tauB = taus[i]
    lineB = B * scfB * np.ones_like(taus)

    plt.loglog(taus, adevs, label='σ')
    plt.loglog(taus, lineB, '--', label='σ_B')
    plt.loglog(tauB, scfB*B, 'o', label='Bias Instability')
    plt.text(tauB, scfB*B, f'0.664B', ha='right', va='bottom')

    plt.title('Allan Deviation with Bias Instability {axis}')
    plt.xlabel('τ (s)')
    plt.ylabel('σ(τ)')
    plt.legend()

    plot_path = os.path.join(PLOT_DIR, f'bias_instability_{axis}.png')
    plt.savefig(plot_path)
    plt.close()

    return tauB, lineB, scfB * B


def plot_allan_angle_random_walk(tau, adev, axis):
    logtau = np.log10(tau)
    logadev = np.log10(adev)
    dlogadev = np.diff(logadev) / np.diff(logtau)

    # Find the index with the slope closest to -0.5
    slope = -0.5
    i = np.argmin(np.abs(dlogadev - slope))

    # Find the y-intercept of the line
    b = logadev[i] - slope * logtau[i]

    # Determine the angle random walk coefficient from the line
    logN = slope * np.log10(1) + b
    N = 10**logN

    # The value at tau=1 for angle random walk (N)
    tauN = 1
    lineN = N / np.sqrt(tau)

    plt.loglog(tau, adev, label='Allan Deviation')
    plt.loglog(tau, lineN, '--', label='Angle Random Walk')

    plt.plot(tauN, N, 'o', label='N')
    plt.title(f'Allan Deviation with Angle Random Walk {axis}')
    plt.xlabel('Tau (s)')
    plt.ylabel('Allan Deviation (σ(τ))')
    plt.legend()
    plt.grid(True)

    plot_path = os.path.join(PLOT_DIR, f'angle_random_walk_{axis}.png')
    plt.savefig(plot_path)
    plt.close()
    return tauN, lineN, N


def plot_allan_rate_random_walk(taus, adevs, axis):
    # Convert to log scale for tau and adev
    logtau = np.log10(taus)
    logadev = np.log10(adevs)
    
    # Calculate the slopes of the log-log Allan deviation
    dlogadev = np.diff(logadev) / np.diff(logtau)
    slope = 0.5  # This is the slope for rate random walk
    
    # Find the index where the slope is closest to the desired slope
    i = np.argmin(np.abs(dlogadev - slope))
    
    # Find the y-intercept of the line
    b = logadev[i] - slope * logtau[i]
    
    # Determine the rate random walk coefficient from the line
    logK = slope * np.log10(3) + b
    K = 10**logK
    
    # Plot the results
    tauK = 3
    lineK = K * np.sqrt(taus/3)
    
    plt.figure()
    plt.loglog(taus, adevs, label='σ')
    plt.loglog(taus, lineK, '--', label='σ_K')
    
    # Plot the rate random walk coefficient K at tau = 3
    plt.loglog(tauK, K, 'o', label='Rate Random Walk')
    
    plt.title('Allan Deviation with Rate Random Walk - ' + axis)
    plt.xlabel('τ (s)')
    plt.ylabel('σ(τ)')
    plt.legend()
    plt.grid(True)

    plot_path = os.path.join(PLOT_DIR, f'rate_random_walk_{axis}.png')
    plt.savefig(plot_path)
    plt.close()
    return tauK, lineK, K


def save_oadev_to_csv(taus, adevs, axis, filepath):
    # Create a DataFrame from taus and adevs
    df = pd.DataFrame({TAU: taus, ADEV: adevs})

    # Save the DataFrame to CSV
    df.to_csv(filepath, index=False)


def plot_allan_variance_and_noise(gyro_data):
    """
    Given gyro data in `rad/s`, for each axis, plot:
    0) Bias Instability
    1) Allen Deviation with Angle Random Walk
    2) Rate Random Walk
    """
    plt.figure(figsize=FIGSIZE)
    for axis, data in gyro_data.items():
        # Prepare the filepath to save or load tau and oadev data.
        filename = f'oadev_{axis}.csv'
        oadev_filepath = os.path.join(f"{DATA_DIR}/{SCENARIO}", filename)

        # If we are computing new Taus and Adevs.
        if COMPUTE_OADEV:
            (taus, adevs, _, _) = oadev(data, rate=1, data_type='freq', taus='all')
            save_oadev_to_csv(taus, adevs, axis, oadev_filepath)
        else:
            # Use preloaded taus and devs from csv
            df = pd.read_csv(oadev_filepath)

            # Assuming your CSV has columns named 'Tau' and 'Adev' for tau and Allan deviation values
            taus = df[TAU].to_numpy()
            adevs = df[ADEV].to_numpy()

        # Plot B (bias instability)
        tauB, lineB, scfB_mult_b = plot_allan_bias_instability(taus, adevs, axis)

        # Plot N (angle random walk)
        tauN, lineN, N = plot_allan_angle_random_walk(taus, adevs, axis)
            
        # Plot K (rate_random_walk)
        tauK, lineK, K = plot_allan_rate_random_walk(taus, adevs, axis)

        plt.figure()
        plt.loglog(taus, adevs, label='$\sigma$ (rad/s)')  # Plot Allan deviation
        plt.loglog(taus, lineN, 'r--', label='$\sigma_N ((rad/s)/\sqrt{Hz})$')  # Plot N
        plt.loglog(taus, lineK, 'g--', label='$\sigma_K ((rad/s)\\sqrt{Hz})$')  # Plot K
        plt.loglog(taus, lineB, 'b--', label='$\sigma_B (rad/s)$')  # Plot B

        # Mark the specific points for N, K, B
        plt.plot(tauN, N, 'ro')  # Mark N
        plt.plot(tauK, K, 'go')  # Mark K
        plt.plot(tauB, scfB_mult_b, 'bo')  # Mark B

        # Add text annotations
        plt.text(tauN, N, 'N')
        plt.text(tauK, K, 'K')
        plt.text(tauB, scfB_mult_b, '0.664B')

        plt.title('Allan Deviation with Noise Parameters')
        plt.xlabel('$\\tau$')
        plt.ylabel('$\sigma(\\tau)$')
        plt.legend()
        plt.grid(True)

        plot_path = os.path.join(PLOT_DIR, f'allan_and_noise_{axis}.png')
        plt.savefig(plot_path)
        plt.close()


def plot_gyro(data):
    """
    Plot Gyro against time for each axis in different fig.
    """
    time = data['elapsed_time'].to_numpy()
    plt.figure(figsize=FIGSIZE)
    
    # Plotting gyro data on three subplots
    for i, axis in enumerate(['x', 'y', 'z'], 1):
        accel_data = data[f'accel_{axis}'].dropna().to_numpy()

        plt.subplot(3, 1, i)
        plt.plot(time, accel_data, label=f'Gyro {axis.upper()}')
        plt.xlabel('Time (s)')
        plt.ylabel('Rotational Rate (rad/s)')
        plt.title(f'Gyro {axis.upper()}')
        plt.legend(loc='upper right')
        plt.grid(True)
    
    plt.subplots_adjust(hspace=0.5)  # 'hspace' controls the height between subplots
    plot_path = os.path.join(PLOT_DIR, 'gyro.png')  
    plt.savefig(plot_path)


def plot_accel(data):
    """
    Plot Acceleration against time for each axis each in different fig.
    """
    time = data['elapsed_time'].to_numpy()
    plt.figure(figsize=FIGSIZE)
    
    # Plotting acceleration data on three subplots
    for i, axis in enumerate(['x', 'y', 'z'], 1):
        accel_data = data[f'accel_{axis}'].dropna().to_numpy()

        plt.subplot(3, 1, i)
        plt.plot(time, accel_data, label=f'Accel {axis.upper()}')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s^2)')
        plt.title(f'Acceleration {axis.upper()}')
        plt.legend(loc='upper right')
        plt.grid(True)
    
    plt.subplots_adjust(hspace=0.5)  # 'hspace' controls the height between subplots
    plot_path = os.path.join(PLOT_DIR, 'accel.png')  # Specify your desired file name
    plt.savefig(plot_path)


def plot_yaw_pitch_roll(data):
    """
    Plot Yaw Pitch Roll against time for each axis in different fig.
    """
    time = data['elapsed_time'].to_numpy()
    plt.figure(figsize=FIGSIZE)
    
    # Plotting yaw, pitch, and roll on three subplots
    for i, axis in enumerate(['yaw', 'pitch', 'roll'], 1):
        # Access the specific axis data and ensure it's in NumPy array form
        axis_data = data[axis].dropna().to_numpy()

        plt.subplot(3, 1, i)
        plt.plot(time, axis_data, label=f'{axis.capitalize()}')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (degrees)')
        plt.title(f'{axis.capitalize()}')
        plt.legend(loc='upper right')
        plt.grid(True)

    plt.subplots_adjust(hspace=0.5)  # 'hspace' controls the height between subplots
    plot_path = os.path.join(PLOT_DIR, 'yaw_pitch_roll.png') 
    plt.savefig(plot_path)


def plot_rotation_histograms(data):
    plt.figure(figsize=(14,12))
    for i, axis in enumerate(['yaw', 'pitch', 'roll'], 1):
        axis_data = data[axis].dropna().to_numpy()
        plt.subplot(3, 1, i)

        plt.hist(axis_data, bins=50, alpha=0.75, label = f'{axis.capitalize()}')
        plt.xlabel(f'{axis.capitalize()} Angle (degrees)')
        plt.ylabel('Frequency (Count)')
        plt.title(f'Histogram of Rotation for {axis.capitalize()}')
        plt.legend(loc='upper right')
        plt.grid(True)
    
    plt.subplots_adjust(hspace=0.5)  # 'hspace' controls the height between subplots
    plot_path = os.path.join(PLOT_DIR, 'rotation_histogram.png') 
    plt.savefig(plot_path)


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

                # Parse each component and convert to the appropriate data type
                vnymrSplit = ""
                if SCENARIO == LOCATION_C:
                    vnymrSplit = msg.data.split(',') 
                else:
                    vnymrSplit = msg.vnymr_read.split(',') 

                # Get the yaw pitch roll (deg)
                yaw = clean_and_convert_to_float(vnymrSplit[VNYMR.Yaw])
                pitch = clean_and_convert_to_float(vnymrSplit[VNYMR.Pitch])
                roll = clean_and_convert_to_float(vnymrSplit[VNYMR.Roll])

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
                    'accel_x': accel_x,
                    'accel_y': accel_y,
                    'accel_z': accel_z,
                    'gyro_x': gyro_x,
                    'gyro_y': gyro_y,
                    'gyro_z': gyro_z,
                }
                writer.writerow(row_dict)

    print("finish convert rosbag to csv")

if __name__ == '__main__':
    if not os.path.exists(PLOT_DIR):
        os.makedirs(PLOT_DIR)

    # Convert bag file to csv.
    if CONVERT_ROSBAG_TO_CSV:
        convert_rosbag_to_csv(bag_filepaths, csv_filepaths)

    # Load data from csv and plot
    data = pd.read_csv(csv_filepaths[0])

    # Plot Allan Variance
    gyro_data = {
        'x': data['gyro_x'].dropna().to_numpy(),
        'y': data['gyro_y'].dropna().to_numpy(),
        'z': data['gyro_z'].dropna().to_numpy(),
    }
    print("Plotting allan variance")
    plot_allan_variance_and_noise(gyro_data)

    # Plotting Gyro, Accel, Yaw Pitch Roll
    print("\nPlotting Gyro Data")
    plot_gyro(data)

    print("\nPlotting Accelerometer Data")
    plot_accel(data)

    print("\nPlotting Yaw, Pitch, Roll")
    plot_yaw_pitch_roll(data)

    print("\nPlotting Rotation Histograms")
    plot_rotation_histograms(data)







