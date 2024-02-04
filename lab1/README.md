# Setup to publish GPS data to serial port.

## Option A: Using GPS puck
List all serial device and find the GPS puck device we want to listen to using:
```
ls /dev/tty*
```

Allow sudo access to the device:
```
sudo chmod 666 /dev/ttyS0
```

## Option B: Using GPS data emitter emulator
To run the emulator program that will write GPS data to serial port, open a new terminal and run the GPS emulator from the `sensor_emulator` directory:
```
python3 serial_emulator.py -f GPS_Chicago.txt
```

It should show the port address as so:
```
The Pseudo device address: /dev/pts/4
```

To see the output in minicom:
```
minicom lab1 -D /dev/pts/4
```

# Capture the GPS messages, publish the messages using ROS node, record, and save to csv.

In `catkins_ws` directory:  

Step 1: Run ROS master.
```
cd ../catkin_ws/
roscore
```

Step 2: In another terminal, run the ROS node that capture the data and publishes for the '/gps' topic:
```
roslaunch gps_driver standalone_driver.launch port:=/dev/pts/3 filename:=chicago
```

for real GPS data from puck:
```
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0 filename:=occlStationary
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0 filename:=occlWalk
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0 filename:=openStationary
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0 filename:=openWalk
```

Step 3: In another terminal check if messages are correctly published for the topic:
```
rostopic echo /gps
```

Step 4: Record the messages published on the topic and save as a bag file.
```
cd data/
rosbag record -O occlStationary/occlStationary.bag /gps
rosbag record -O occlWalk/occlWalk.bag /gps
rosbag record -O openStationary/openStationary.bag /gps
rosbag record -O openWalk/openWalk.bag /gps
rosbag record -O chicago/chicago.bag /gps
```

Step 5: Exit the publisher node and the record program.

Step 6: To save bag file as csv file, cd into the `data` directory, change the filename for the csv file,
uncomment the function call for `convert_rosbag_to_csv(bag_file, csv_file)`
 then run:
```
python3 dataUtil.py
```

(Optional) To to replay as if we are publishing again:
```
rosbag info openWalk/openWalk.bag.bag 
```

Then we can execute `rostopic echo gppga_topic` again we will see whats being published.
# Autograder
To run autograder, copy the autograder directory inside EECE5554/ directory then run:
```
bash script.sh freecode23
```

To manually start screen session for emulator with arguments that is inside the script:
```
screen -S emulator -dm bash -c "echo $pwd;python3 serial_emulator.py -f gps-data.txt; echo $pwd"
```
To start screen session for ros node:
```
screen -S ros_node -dm roslaunch gps_driver standalone_driver.launch port:=/dev/pts/4
```

To remove all screen session:
```
screen -ls | grep 'ros_node' | awk -F '.' '{print $1}' | xargs -I {} screen -S {} -X quit
screen -ls | grep 'emulator' | awk -F '.' '{print $1}' | xargs -I {} screen -S {} -X quit
```
### Autograder expected output:
Time Stamp for Seconds :  9298
Time Stamp for Nano-Seconds :  230000000.0
latitude :  34.02019816666667
longitude :  -118.41129950000001
easting :  369695.4373543182
northing :  3765293.4953880184
zone :  11
letter :  S
header :  GPS1_Frame
hdop: 1.0
altitude: 0

# Data capture

1. In occluded spot near building or trees: 5 mins stationary (behind snell)
2. Also do walking here.
3. In open spot without builidng within 10 m radius: 5 mins stationary.

