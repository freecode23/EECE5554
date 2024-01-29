import rosbag
import csv

# Replace 'your_bag_file.bag' with the path to your ROS bag file
bag_file = 'gpgga.bag'

# Replace 'output.csv' with the desired output CSV file name
output_csv_file = 'output.csv'

# Function to convert a ROS bag to CSV
def convert_rosbag_to_csv(bag_file, output_csv_file):
    with rosbag.Bag(bag_file, 'r') as bag, open(output_csv_file, 'w', newline='') as csvfile:
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

convert_rosbag_to_csv(bag_file, output_csv_file)
