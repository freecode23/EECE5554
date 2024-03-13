# Introduction
The aim of imu library is to create a program that ingest IMU VNYMR data from a IMU puck, process the data to get the UTM information, and convert the data type to ROS custom message so that it can published as ROS topic. We will also be saving the ROS messages logs to a bag file so we can convert it to .csv file and perform various data analysis.
# 1. Preliminiary setup to publish IMU data to serial port.
## Option A: Using IMU sensor
List all serial device and find the IMU puck device we want to listen to using:
```
ls /dev/tty*
```

Allow sudo access to the device:
```
sudo chmod 666 /dev/ttyS0
```

Make sure latency is set to 1 ms:
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## Option B: Using IMU data emitter emulator
To run the emulator program that will write IMU data to serial port, open a new terminal and run the IMU emulator from the `sensor_emulator` directory:
```
python3 serial_emulator.py --file imu_data.txt --device_type imu --loop "yes" --VN_reg b'$VNWRG,07,40*XX'
python3 serial_emulator.py --file imu_data.txt --device_type imu --loop "no"
```

It should show the port address as so:
```
The Pseudo device address: /dev/pts/4
```

To see the output in minicom:
```
minicom lab1 -D /dev/pts/4
```

# 2. Capture the IMU messages, publish the messages using ROS node, record, and save to csv.

In `catkins_ws` directory:  

Step 1: Run ROS master and build the workspace.
```
cd ../catkin_ws/
roscore
catkin_make
source devel/setup.bash
```

Step 2: In another terminal, run the ROS node that capture the data and publishes for the '/imu' topic:
for emulator data:
```
roslaunch vn_driver driver.launch port:=/dev/pts/4 filename:=dead_reckoning
```

for real IMU data from puck:
```
roslaunch vn_driver driver.launch port:=/dev/ttyUSB0 filename:=dead_reckoning
```

Step 3: In another terminal check if messages are correctly published for the topic:
```
rostopic echo /imu
```

Step 4: Record the messages published on the topic and save as a bag file.
```
cd data/
rosbag record -O dead_reckoning/dead_reckoning.bag /imu
```

Step 5: Exit the publisher node and the record program.

Step 6: To save bag file as csv file, cd into the `analysis` directory, change the filename for the csv file,
uncomment the function call for `convert_rosbag_to_csv(bag_file, csv_file)`
 then run:
```
python3 dataUtil.py
```

(Optional) To to replay as if we are publishing again:
```
rosbag info openWalk/openWalk.bag
```

Then we can execute `rostopic echo gppga_topic` again we will see whats being published.
