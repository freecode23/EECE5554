import rosbag
import csv
import pandas as pd
import plotly.graph_objects as go
import os
import numpy as np
from plotly.subplots import make_subplots
import plotly.express as px


IMAGE_EXTENSION = "png"

# Scenario selections:
# Chicago test scenario
CHICAGO = "chicago"

# Regular GPS scenario
STATIONARY = "Stationary"
WALK = "Walk"

# RTK scenario
RTK = "RTK"
STATIONARY_RTK = f"Stationary{RTK}"
WALK_RTK = f"walking{RTK}"

scenario = WALK_RTK
# Replace filename or scenario with the desired name before running the program.
# For converting bag file to csv file.
if scenario == CHICAGO or scenario == WALK_RTK:
    filename = scenario
    csv_filepath = f'{filename}/{filename}.csv'
    csv_filepaths = [csv_filepath]
    bag_filepath = f'{filename}/{filename}.bag'

# Get both open and occluded situation.
else:
    # Replace scenario for plotting.
    filename = f'open{scenario}'
    filename2 = f'occl{scenario}'
 
    # Get file paths for open and occluded. 
    csv_filepath = f'{filename}/{filename}.csv'
    csv_filepath2 = f'{filename2}/{filename2}.csv'
    csv_filepaths = [csv_filepath, csv_filepath2]


# Function to convert a ROS bag to CSV.
def convert_rosbag_to_csv(bag_filepath, csv_filepath):
    read_format = 'gpgga_read'
    if RTK in bag_filepath:
        read_format = 'gngga_read'

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
            if RTK in bag_filepath:
                row_dict[read_format] = msg.gngga_read
                row_dict['fix_quality'] = msg.fix_quality
            else:
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

# Function to calculate distance from a point to a line
# This is to compute the RMSE for walking scenario.
def point_to_line_dist(point, slope, intercept):
    px, py = point

    # Calculate distance using the formula for perpendicular distance from a point to a line:
    # |Ax + By + C| / sqrt(A^2 + B^2) where line is Ax + By + C = 0
    # For y = mx + b, A = m, B = -1, and C = b
    dist = abs(slope * px - py + intercept) / (np.sqrt(slope**2 + 1))
    return dist


def calculate_2DRMS_from_perpendicular_distances(perpendicular_distances):
    # Calculate the square of the distances
    distances_sq = perpendicular_distances**2
    
    # Calculate the mean of the squared distances
    mean_distances_sq = np.mean(distances_sq)
    
    # Calculate the DRMS (Distance Root Mean Square)
    drms = np.sqrt(mean_distances_sq)
    
    # 2DRMS is two times the DRMS
    drms_2 = drms * 2
    return drms_2

# Function to calculate DRMS
def calculate_2DRMS(df: pd.DataFrame):
    # Square of differences
    df['easting_diff_sq'] = df['easting_diff']**2
    df['northing_diff_sq'] = df['northing_diff']**2
    
    # Mean of squared differences
    mean_easting_diff_sq = df['easting_diff_sq'].mean()
    mean_northing_diff_sq = df['northing_diff_sq'].mean()
    
    # DRMS is the square root of the mean of the sum of squared differences
    drms = np.sqrt(mean_easting_diff_sq + mean_northing_diff_sq)
    return drms * 2

def plotNorthingEasting(csv_filepaths: list, plot_filepath: str, scenario: str, isLineOfBestFit: bool):
    # Initialize figure
    fig = make_subplots()

    # Initialize text for the text box
    textbox_content = ""

    # Loop over each CSV file path
    for i, csv_file in enumerate(csv_filepaths):
        # 1. Load data
        df = pd.read_csv(csv_file)

        # Get normalized northing and easting columns.
        df, first_point_easting, first_point_northing = getNormalizedNorthingEasting(df)

        # Calculate the centroids of the normalized data
        centroid_easting = df['easting_normalized'].mean()
        centroid_northing = df['northing_normalized'].mean()

        # Subtract centroid from each data point
        df['easting_diff'] = df['easting_normalized'] - centroid_easting
        df['northing_diff'] = df['northing_normalized'] - centroid_northing

        if RTK not in scenario:
            scenario = scenario.capitalize()

        # Get the base file name without the scenario name.
        openOrOc = os.path.splitext(os.path.basename(csv_file))[0].replace(scenario, "")
        df['openOrOc'] = openOrOc

        # Create textbox content.
        zone = df['zone'][0]
        letter = df['letter'][0]
        textbox_content += f"\
        {openOrOc}<br>\
        Zone:{zone}<br>\
        Letter:{letter}<br>\
        Offset: <br>\
        East: {(first_point_easting/1000.0):.2f} km<br>\
        North:{(first_point_northing/1000.0):.2f} km<br>"

        # Plot scatter points.
        fig.add_trace(go.Scatter(
            x=df['easting_diff'], y=df['northing_diff'], 
            mode='markers', name=openOrOc))
        
        # Calculate error metrics.
        drms2_value = 0
        if isLineOfBestFit:
            # Compute and overlay line of best fit.
            m, b = np.polyfit(df['easting_diff'], df['northing_diff'], 1)

            # Plot.
            fig.add_trace(go.Scatter(
                x=df['easting_diff'], 
                y=m*df['easting_diff'] + b, 
                mode='lines', 
                name=f'{openOrOc} Best Fit'))
            
            # Calculate perpendicular distances from each data point to the line of best fit.
            perpendicular_distances = df.apply(
                lambda row: point_to_line_dist((row['easting_diff'], row['northing_diff']), m, b), axis=1)

            # Error for walking: Calculate RMSE of these perpendicular distances
            drms2_value = calculate_2DRMS_from_perpendicular_distances(perpendicular_distances)
        else:
            # Error for stationary: Calculate 2DRMS:
            drms2_value = calculate_2DRMS(df)
    
        print(f"The 2DRMS value for {scenario} {openOrOc} is: {drms2_value}")
        
    # Customize the plot
    fig.update_layout(
        title=f"{scenario} Northing vs Easting (Differences from Centroid)",
        xaxis_title="Easting Difference from Centroid (m)",
        yaxis_title="Northing Difference from Centroid (m)",
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


def plotStationaryHistogram(csv_filepaths: list):

    for i, csv_file in enumerate(csv_filepaths):
        # Load data
        df = pd.read_csv(csv_file)

        # Get normalized northing and easting columns.
        df, _, _ = getNormalizedNorthingEasting(df)

        # Calculate the centroids of the normalized data
        centroid_easting = df['easting_normalized'].mean()
        centroid_northing = df['northing_normalized'].mean()

        # Calculate Euclidean distance from each of normalized points to the centroid.
        df['euclidean_distance'] = np.sqrt((df['easting_normalized'] - centroid_easting)**2 + (df['northing_normalized'] - centroid_northing)**2)

        # Extract the file name from the file path for the plot title.
        filename = os.path.splitext(os.path.basename(csv_file))[0]

        # Create a histogram plot.
        fig = px.histogram(df, x='euclidean_distance', 
                           title=f'Stationary Histogram for {filename}')

        # Customize the plot.
        fig.update_layout(
            xaxis_title="Euclidean Distance to Centroid (meters)",
            yaxis_title="Count",
            plot_bgcolor="white"
        )

        # Save the plot as a PNG file
        fig.write_image(f"{filename}EuclDistanceHistogram.{IMAGE_EXTENSION}")

if __name__ == '__main__':
    if scenario == CHICAGO:
        # Convert bag file to csv.
        convert_rosbag_to_csv(bag_filepath, csv_filepath)


        # Plot Northing and Easting data.
        plotNorthingEasting(csv_filepaths, 
                        plot_filepath=f'{scenario.lower()}NorthingEasting.{IMAGE_EXTENSION}', 
                        scenario=scenario, 
                        isLineOfBestFit=False)
    
        exit(0)
    
    # Plot
    # For stationary, we don't need line of best fit.
    isLineOfBestFit = True
    if STATIONARY in scenario:
        isLineOfBestFit = False

        # 1. Plot Histogram.
        plotStationaryHistogram(csv_filepaths)

    print("FILEPATHS=", csv_filepaths)
    # 2. Plot Northing Easting difference.
    plotNorthingEasting(csv_filepaths, 
                        plot_filepath=f'{scenario.lower()}NorthingEasting.{IMAGE_EXTENSION}', 
                        scenario=scenario, 
                        isLineOfBestFit=isLineOfBestFit)

    # 3. Plot Altitude vs time.
    plotAltitudeVsTime(csv_filepaths, 
                    plot_filepath=f'{scenario.lower()}AltitudeTime.{IMAGE_EXTENSION}', 
                    scenario=scenario)



