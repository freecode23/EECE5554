# Setup to publish GPS data to serial port.

## Option A: Using GPS puck
list all serial device and find the GPS puck device:
```
ls /dev/tty*
```

Allow sudo access the device:
```
sudo chmod 666 /dev/ttyS0
```

## Option B: Using GPS data emitter emulator
To run the emulator program that will write GPS data to serial port, open a new terminal and run the GPS emulator from the `sensor_emulator` directory:
```
python3 serial_emulator.py -f GPS_Chicago.txt
```

It should show:
```
The Pseudo device address: /dev/pts/4
```

To see the output in minicom:
```
minicom lab1 -D /dev/pts/4
```

# To capture the GPS messages and publish using ROS node, record, and save to csv:

In `catkins_ws` directory:
Step 1: Run ROS master.
```
cd ../catkin_ws/
roscore
```

Step 2: In another terminal, run the ROS node that capture the data and publishes for the gpgga topic:
```
rosrun gps_driver standalone_driver.py
```
or
```
roslaunch gps_driver standalone_driver.launch port:=/dev/pts/4 filename=fileA.txt
```
for real GPS data from puck:
```
roslaunch gps_driver standalone_driver.launch port:=/dev/ttyUSB0

Step 3: In another terminal check if messages are correctly published for the topic:
```
rostopic echo gpgga_topic
```

Step 4: Record as bag file.
```
cd data/
rosbag record -O gpgga.bag gpgga_topic
```

(Optional) To to replay as if we are publishing again:
```
rosbag info 2024-01-28-22-08-54.bag 
```
Then we can execute `rostopic echo gppga_topic` again we will see whats being published.

Step 5: Exit the publisher node and the record program.

Step 6: Save as csv file
```
python3 rosbagUtil.py
```


To run autograder bash script.sh <repo link>