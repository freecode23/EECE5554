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
CHICAGO = "chicago"

scenario = CHICAGO
# Replace filename or scenario with the desired name before running the program.
# For converting bag file to csv file.

if scenario == CHICAGO:
    filename = scenario
    csv_filepath = f'{filename}/{filename}.csv'
    csv_filepaths = [csv_filepath]
    bag_filepath = f'{filename}/{filename}.bag'

else:
    # Replace scenario for plotting.
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
    # Initialize figure
    fig = make_subplots()

    # Initialize text for the text box
    textbox_content = ""

    # Loop over each CSV file path
    for i, csv_file in enumerate(csv_filepaths):
        # Load data
        df = pd.read_csv(csv_file)
        print(df)
        # Subtract the first data point from each data point in the dataset
        first_point_easting = df.iloc[0]['utm_easting']
        first_point_northing = df.iloc[0]['utm_northing']
        df['easting_normalized'] = df['utm_easting'] - first_point_easting
        df['northing_normalized'] = df['utm_northing'] - first_point_northing

        # Calculate the centroids of the normalized data
        centroid_easting = df['easting_normalized'].mean()
        centroid_northing = df['northing_normalized'].mean()

        # Subtract centroid from each data point
        df['easting_diff'] = df['easting_normalized'] - centroid_easting
        df['northing_diff'] = df['northing_normalized'] - centroid_northing

        # Extract the file name, zone, and letter
        situation = os.path.splitext(os.path.basename(csv_file))[0].replace(scenario.capitalize(), "")
        df['situation'] = situation
        zone = df['zone'][0]
        letter = df['letter'][0]
        # Append centroid info to the text box content
        textbox_content += f"\
        {situation}<br>\
        Zone:{zone}<br>\
        Letter:{letter}<br>\
        Offset: <br>\
        East: {(first_point_easting/1000.0):.2f} m<br>\
        North:{(first_point_northing/1000.0):.2f} m<br>"

        # Add data to plot
        if isLineOfBestFit:
            # Compute and plot line of best fit
            m, b = np.polyfit(df['easting_diff'], df['northing_diff'], 1)
            fig.add_trace(go.Scatter(
                x=df['easting_diff'], y=m*df['easting_diff'] + b, 
                mode='lines', name=f'{situation} Best Fit'))
        
        # Plot scatter points
        fig.add_trace(go.Scatter(
            x=df['northing_diff'], y=df['easting_diff'], 
            mode='markers', name=situation))

    # Customize the plot
    fig.update_layout(
        title=f"{scenario} Northing vs Easting (Differences from Centroid)",
        xaxis_title="Northing Difference from Centroid (m)",
        yaxis_title="Easting Difference from Centroid (m)",
        plot_bgcolor="white",
        annotations=[
            dict(
                text=textbox_content, # Textbox content
                align='left',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=1.05,
                y=1.05, # Position above the plot
                bordercolor='black',
                borderwidth=1
            )
        ]
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

        # Extract the file name from the file path and use it as the situation name
        filename = os.path.splitext(os.path.basename(csv_file))[0]
        df['situation'] = filename

        # Append to the combined DataFrame
        combined_df = pd.concat([combined_df, df])

    # Plot Altitude vs Time
    fig = make_subplots()
    for situation in combined_df['situation'].unique():
        dataset = combined_df[combined_df['situation'] == situation]
        fig.add_trace(go.Scatter(x=dataset['stamp'], y=dataset['altitude'], mode='markers', name=situation))

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
    convert_rosbag_to_csv(bag_filepath, csv_filepath)

    # 2. Plot Northing and Easting Chicago data.
    plotNorthingEasting(csv_filepaths, 
                    plot_filepath=f'{scenario.lower()}NorthingEasting.{IMAGE_EXTENSION}', 
                    scenario=scenario, 
                    isLineOfBestFit=False)
    


    # # 3. For stationary, we don't need line of best fit.
    # isLineOfBestFit = True
    # if scenario == STATIONARY:
    #     isLineOfBestFit = False

    #     # Plot Histogram.
    #     plotStationaryHistogram(csv_filepaths)


    # # 4. Plot all.
    # plotNorthingEasting(csv_filepaths, 
    #                     plot_filepath=f'{scenario.lower()}NorthingEasting.{IMAGE_EXTENSION}', 
    #                     scenario=scenario, 
    #                     isLineOfBestFit=isLineOfBestFit)

    # plotAltitudeVsTime(csv_filepaths, 
    #                 plot_filepath=f'{scenario.lower()}AltitudeTime.{IMAGE_EXTENSION}', 
    #                 scenario=scenario)



