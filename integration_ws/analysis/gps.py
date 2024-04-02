import rosbag
import csv
import pandas as pd
import plotly.graph_objects as go
import os
import numpy as np
from plotly.subplots import make_subplots
from scipy.spatial.distance import euclidean



IMAGE_EXTENSION = "png"
DATA_DIR = "../data"
PLOT_DIR = f"{DATA_DIR}/plots"

CIRCLE = f"circle"
TOWN = f"town"

# Replace with True if we want to first convert the bag file to csv.
CONVERT_ROSBAG_TO_CSV = True
scenario = TOWN

# Get the bag and csv filepath.
filename = scenario
bag_filepath = f'{DATA_DIR}/{filename}/town_gps.bag'
bag_filepaths = [bag_filepath]

csv_filepath_gps = f'{DATA_DIR}/{filename}/town_gps.csv'
csv_filepath_imu = f'{DATA_DIR}/{filename}/town_imu.csv'
csv_filepaths = [csv_filepath_gps, csv_filepath_imu]


# Function to convert a ROS bag to CSV.
def convert_rosbag_to_csv(bag_filepaths, csv_filepaths):

    for i in range(len(bag_filepaths)):
        bag_filepath = bag_filepaths[i]
        csv_filepath = csv_filepaths[i]

        read_format = 'gpgga_read'

        with rosbag.Bag(bag_filepath, 'r') as bag, open(csv_filepath, 'w', newline='') as csvfile:
            fieldnames = ['seq', 'stamp', 'frame_id', 'latitude', 
                        'longitude', 'altitude', 'utm_easting', 
                        'utm_northing', 'zone', 'letter', 
                        'hdop', read_format, 'fix_quality']
            
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for topic, msg, t in bag.read_messages():
                row_dict = {
                        'seq': msg.header.seq,
                        'stamp': msg.header.stamp.to_sec(),
                        'frame_id': msg.header.frame_id,
                        'latitude': msg.latitude,
                        'longitude': msg.longitude,
                        'altitude': msg.altitude,
                        'utm_easting': msg.utm_easting,
                        'utm_northing': msg.utm_northing,
                        'zone': msg.zone,
                        'letter': msg.letter,
                        'hdop': msg.hdop,
                    }
                row_dict[read_format] = msg.gpgga_read


                # Change the topic name to match your specific topic
                if topic == '/gps':
                    writer.writerow(row_dict)

def getNormalizedNorthingEasting(df: pd.DataFrame):
    # Subtract the first data point from each data point in the dataset
    first_point_easting = df.iloc[0]['utm_easting']
    first_point_northing = df.iloc[0]['utm_northing']
    df['easting_normalized'] = df['utm_easting'] - first_point_easting
    df['northing_normalized'] = df['utm_northing'] - first_point_northing

    return df, first_point_easting, first_point_northing


def plotNorthingEasting(plot_filepath: str, scenario: str):
    # Initialize figure
    fig = make_subplots()
    textbox_content = ""
    df_gps = pd.read_csv(csv_filepath_gps)
    df_imu = pd.read_csv(csv_filepath_imu)

    # Get normalized northing and easting columns for gps
    df_gps, first_point_easting, first_point_northing = getNormalizedNorthingEasting(df_gps)
    df_imu, imu_first_point_easting, imu_first_point_northing = getNormalizedNorthingEasting(df_imu)

    # Iterate through GPS timestamps and find the closest IMU timestamps within the window
    combined_data_list = []

    time_window = pd.Timedelta(seconds=0.25)
    df_gps['timestamp'] = pd.to_datetime(df_gps['stamp'], unit='s')
    df_imu['timestamp'] = pd.to_datetime(df_imu['stamp'], unit='s')
    for index, gps_row in df_gps.iterrows():
        # Find the index of the closest IMU timestamp to the current GPS timestamp
        closest_index = (df_imu['timestamp'] - gps_row['timestamp']).abs().idxmin()
        closest_imu_row = df_imu.iloc[closest_index]
        time_diff = abs(closest_imu_row['timestamp'] - gps_row['timestamp'])

        # Check if the closest timestamp is within the time window
        if time_diff <= time_window:
            # Combine the data for this timestamp
            combined_data = {
                'gps_timestamp': gps_row['timestamp'],
                'gps_northing': gps_row['northing_normalized'],
                'gps_easting': gps_row['easting_normalized'],
                'imu_timestamp': closest_imu_row['timestamp'],
                'imu_northing': closest_imu_row['northing_normalized'],  # Assuming this column exists
                'imu_easting': closest_imu_row['easting_normalized']      # Assuming this column exists
            }
            # Append the combined data to the synced DataFrame
            combined_data_list.append(combined_data)

    synced_data = pd.DataFrame(combined_data_list)
    print(synced_data)

    # Create filename
    scenario = scenario.capitalize()
    suffix = os.path.splitext(os.path.basename(csv_filepath_gps))[0].replace(scenario, "")

    # Create textbox content.
    zone = df_gps['zone'][0]
    letter = df_gps['letter'][0]
    textbox_content += f"\
    {suffix}<br>\
    Zone:{zone}<br>\
    Letter:{letter}<br>\
    Offset: <br>\
    East: {(first_point_easting/1000.0):.2f} km<br>\
    North:{(first_point_northing/1000.0):.2f} km<br>"

    # Plot GPS data points
    fig.add_trace(go.Scatter(
        x=synced_data['gps_easting'], 
        y=synced_data['gps_northing'], 
        mode='markers', 
        name='GPS Data', 
        marker=dict(color='rgba(255, 0, 0, .8)')
    ))

    # Plot IMU data points from the synchronized DataFrame
    scaling_factor = 1.8
    fig.add_trace(go.Scatter(
        x=synced_data['imu_easting'] * scaling_factor, 
        y=synced_data['imu_northing'] * scaling_factor, 
        mode='markers', 
        name='IMU Data', 
        marker=dict(color='rgba(0, 0, 255, .8)')
    ))

    # Customize the plot
    title=f"{scenario} Northing vs Easting Comparison"
    xaxis_title="Easting (m)"
    yaxis_title="Northing (m)"
    fig.update_layout(
        title=title,
        xaxis_title=xaxis_title,
        yaxis_title=yaxis_title,
        legend_title="Source",
        plot_bgcolor="white"
    )

    # Save the plot as a PNG file
    fig.write_image(plot_filepath)

    # Create a new column for the distance between GPS and IMU points
    synced_data['distance'] = synced_data.apply(
        lambda row: euclidean(
            (row['gps_easting'], row['gps_northing']),
            (row['imu_easting'], row['imu_northing'])
        ),
        axis=1
    )

    # Find periods where distance is less than or equal to 2 meters
    synced_data['close_match'] = synced_data['distance'] <= 2

    # Now, find continuous periods where 'close_match' is True
    matching_periods = []
    start_time = None
    end_time = None
    for i, row in synced_data.iterrows():
        if row['close_match']:
            if start_time is None:
                start_time = row['gps_timestamp']
            end_time = row['gps_timestamp']
        else:
            if start_time is not None and end_time is not None:
                matching_periods.append((start_time, end_time))
                start_time = None
                end_time = None
    # Don't forget to check the last period if it ends at the end of the data
    if start_time is not None and end_time is not None:
        matching_periods.append((start_time, end_time))

    for period in matching_periods:
        print(f"Close match from {period[0]} to {period[1]}")

    # Calculate the duration of each matching period
    matching_durations = [(end - start).total_seconds() for start, end in matching_periods]

    # Sum the durations to find the total matching time
    total_matching_time = sum(matching_durations)

    # Now to estimate how long you could navigate without a fix
    # Assuming that your initial position is accurately known and that IMU drift accumulates over time,
    # look at the longest period of close match to get an idea of the maximum time
    max_continuous_matching_time = max(matching_durations, default=0)

    print("max_continueous_matching_time=", max_continuous_matching_time)



def plotAltitudeVsTime(csv_filepaths: list, plot_filepath: str, scenario: str):
    # Initialize an empty DataFrame for combined data
    combined_df = pd.DataFrame()

    # Loop over each CSV file path
    for csv_file in csv_filepaths:
        # Load data
        df = pd.read_csv(csv_file)

        # Extract the file name from the file path and use it as the openOrOc name
        filename = os.path.splitext(os.path.basename(csv_file))[0]
        df['openOrOccluded'] = filename

        # Append to the combined DataFrame
        combined_df = pd.concat([combined_df, df])

    # Plot Altitude vs Time for open and occluded.
    fig = make_subplots()
    for openOrOc in combined_df['openOrOccluded'].unique():
        dataset = combined_df[combined_df['openOrOccluded'] == openOrOc]
        fig.add_trace(go.Scatter(x=dataset['stamp'], 
                                 y=dataset['altitude'], 
                                 mode='markers', 
                                 name=openOrOc))

    # Customize the plot
    fig.update_layout(
        title=f"{scenario} Altitude vs Time",
        xaxis_title="Time (seconds since epoch)",
        yaxis_title="Altitude (meters)",
        plot_bgcolor="white"
    )

    # Save the plot as a PNG file
    fig.write_image(plot_filepath)



if __name__ == '__main__':
    if not os.path.exists(f'{PLOT_DIR}/{filename}'):
        os.makedirs(f'{PLOT_DIR}/{filename}')
    # Convert bag file to csv.
    # if CONVERT_ROSBAG_TO_CSV:
    #     convert_rosbag_to_csv(bag_filepaths, csv_filepaths)

    # Plot Northing and Easting data.
    plotNorthingEasting(
                    plot_filepath=f'{PLOT_DIR}/{filename}/{scenario.lower()}NorthingEasting_IMU_GPS.{IMAGE_EXTENSION}', 
                    scenario=scenario)
    
