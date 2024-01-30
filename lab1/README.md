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
roslaunch gps_driver standalone_driver.launch port:=/dev/pts/3 filename:=fileA
```

for real GPS data from puck:
```
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0 filename:=fileA
```

Step 3: In another terminal check if messages are correctly published for the topic:
```
rostopic echo /gps
```

Step 4: Record the messages published on the topic and save as a bag file.
```
cd data/
rosbag record -O gpgga.bag /gps
```

(Optional) To to replay as if we are publishing again:
```
rosbag info 2024-01-28-22-08-54.bag 
```
Then we can execute `rostopic echo gppga_topic` again we will see whats being published.

Step 5: Exit the publisher node and the record program.

Step 6: To save bag file as csv file, cd into the `data` directory, then run:
```
python3 rosbagUtil.py
```

# Autograder
To run autograder, cd into the autograder directory inside lab1 directory then run:
```
bash script.sh freecode23
```

To start screen session for emulator with arguments
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
