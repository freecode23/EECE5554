import rosbag
import csv
import pandas as pd
import plotly.graph_objects as go
import os
import numpy as np
from plotly.subplots import make_subplots
import plotly.express as px

STATIONARY = "Stationary"
WALK = "Walk"
IMAGE_EXTENSION = "png"

# Replace filename or scenario with the desired name before running the program.
# For converting bag file to csv file.
# filename = 'chicago'
# csv_filepath = f'{filename}/{filename}.csv'
# csv_filepaths = [csv_filepath]
# bag_filepath = f'{filename}/{filename}.bag'

# Replace scenario for plotting.
scenario = STATIONARY
filename = f'open{scenario}'
filename2 = f'occl{scenario}'
csv_filepath = f'{filename}/{filename}.csv'
csv_filepath2 = f'{filename2}/{filename2}.csv'
csv_filepaths = [csv_filepath, csv_filepath2 ]


# Function to convert a ROS bag to CSV.
def convert_rosbag_to_csv(bag_filepath, csv_filepath):
    with rosbag.Bag(bag_filepath, 'r') as bag, open(csv_filepath, 'w', newline='') as csvfile:
        fieldnames = ['seq', 'stamp', 'frame_id', 'latitude', 'longitude', 'altitude', 'utm_easting', 'utm_northing', 'zone', 'letter', 'hdop', 'gpgga_read']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for topic, msg, t in bag.read_messages():
            # Change the topic name to match your specific topic
            if topic == '/gps':
                writer.writerow({
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
                    'gpgga_read': msg.gpgga_read
                })


def plotNorthingEasting(csv_filepaths: list, plot_filepath: str, scenario: str, isLineOfBestFit: bool):

    # Initialize an empty DataFrame for combined data
    combined_df = pd.DataFrame()

    # Loop over each CSV file paths
    for i, csv_file in enumerate(csv_filepaths):
        # Load data
        df = pd.read_csv(csv_file)

        # Calculate the centroids
        centroid_easting = df['utm_easting'].mean()
        centroid_northing = df['utm_northing'].mean()

        # Subtract centroid from each data point
        df['easting_diff'] = df['utm_easting'] - centroid_easting
        df['northing_diff'] = df['utm_northing'] - centroid_northing
        
        # Extract the file name from the file path and use it as the source name
        filename = os.path.splitext(os.path.basename(csv_file))[0]
        df['source'] = filename
        print("filename=", filename)

        # Append to the combined DataFrame
        combined_df = pd.concat([combined_df, df])

    # Plot the differences for all points or just line of best fit.
    fig = make_subplots()
    for source in combined_df['source'].unique():
        dataset = combined_df[combined_df['source'] == source]

        # Add trace for scatter plot
        fig.add_trace(go.Scatter(x=dataset['easting_diff'], y=dataset['northing_diff'], mode='markers', name=source))

        if isLineOfBestFit:
            # Compute line of best fit
            m, b = np.polyfit(dataset['easting_diff'], dataset['northing_diff'], 1)
            # Add trace for line of best fit
            fig.add_trace(go.Scatter(x=dataset['easting_diff'], y=m*dataset['easting_diff']+b, mode='lines', name=f'{source} Best Fit'))
        

    # Customize the plot
    fig.update_layout(
        title=f"{scenario} Northing vs Easting (Differences from Centroid)",
        xaxis_title="Easting Difference from Centroid",
        yaxis_title="Northing Difference from Centroid",
        plot_bgcolor="white"
    )

    # Save the plot as a PNG file
    fig.write_image(plot_filepath)


def plotAltitudeVsTime(csv_filepaths: list, plot_filepath: str, scenario: str):
    # Initialize an empty DataFrame for combined data
    combined_df = pd.DataFrame()

    # Loop over each CSV file path
    for csv_file in csv_filepaths:
        # Load data
        df = pd.read_csv(csv_file)

        # Extract the file name from the file path and use it as the source name
        filename = os.path.splitext(os.path.basename(csv_file))[0]
        df['source'] = filename

        # Append to the combined DataFrame
        combined_df = pd.concat([combined_df, df])

    # Plot Altitude vs Time
    fig = make_subplots()
    for source in combined_df['source'].unique():
        dataset = combined_df[combined_df['source'] == source]
        fig.add_trace(go.Scatter(x=dataset['stamp'], y=dataset['altitude'], mode='markers', name=source))

    # Customize the plot
    fig.update_layout(
        title=f"{scenario} Altitude vs Time",
        xaxis_title="Time (seconds since epoch)",
        yaxis_title="Altitude (meters)",
        plot_bgcolor="white"
    )

    # Save the plot as a PNG file
    fig.write_image(plot_filepath)


def plotStationaryHistogram(csv_filepaths: list):

    for i, csv_file in enumerate(csv_filepaths):
        # Load data
        df = pd.read_csv(csv_file)

        # Calculate the centroid
        centroid_easting = df['utm_easting'].mean()
        centroid_northing = df['utm_northing'].mean()

        # Calculate Euclidean distance from each point to the centroid
        df['euclidean_distance'] = np.sqrt((df['utm_easting'] - centroid_easting)**2 + (df['utm_northing'] - centroid_northing)**2)

        # Extract the file name from the file path for the plot title
        filename = os.path.splitext(os.path.basename(csv_file))[0]

        # Create a histogram plot
        fig = px.histogram(df, x='euclidean_distance', title=f'Stationary Histogram for {filename}')

        # Customize the plot
        fig.update_layout(
            xaxis_title="Euclidean Distance to Centroid",
            yaxis_title="Count",
            plot_bgcolor="white"
        )

        # Save the plot as a PNG file
        fig.write_image(f"{filename}EuclDistancteHistogram.{IMAGE_EXTENSION}")

if __name__ == '__main__':
    # 1. Convert bag file to csv.
    # convert_rosbag_to_csv(bag_filepath, csv_filepath)

    # 2. Plot Northing and Easting Chicago data.
    # plotNorthingEasting(csv_filepaths, plot_filepath)

    isLineOfBestFit = True

    # 3. For stationary, we don't need line of best fit.
    if scenario == STATIONARY:
        isLineOfBestFit = False

        # Plot Histogram.
        plotStationaryHistogram(csv_filepaths)


    # 4. Plot all.
    plotNorthingEasting(csv_filepaths, 
                        plot_filepath=f'{scenario.lower()}NorthingEasting.{IMAGE_EXTENSION}', 
                        scenario=scenario, 
                        isLineOfBestFit=isLineOfBestFit)

    plotAltitudeVsTime(csv_filepaths, 
                    plot_filepath=f'{scenario.lower()}AltitudeTime.{IMAGE_EXTENSION}', 
                    scenario=scenario)



