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

## Option B: Using IMU and GPS data emitter emulator
To run the emulator program that will write IMU data to serial port, open a new terminal and run the IMU emulator from the `sensor_emulator` directory:
```
python3 serial_emulator.py --file imu_data.txt --device_type imu --loop "yes" --VN_reg b'$VNWRG,07,40*XX'
python3 serial_emulator.py --file imu_data.txt --device_type imu --loop "no"
python3 serial_emulator.py --file gps_data.txt --device_type gps --loop "yes"

```

It should show the port address as so:
```
The Pseudo device address: /dev/pts/4
```

To see the output in minicom:
```
minicom lab1 -D /dev/pts/4
```

# 2. Capture the IMU and GPS messages, publish the messages using ROS node, record, and save to csv:

In `integration_ws` directory:  

Step 1: Run ROS master and build the workspace.
```
cd ../integration_ws/
roscore
catkin_make
source devel/setup.bash
```

Step 2. roslaunch the gps and imu also from the`integration_ws` dir.
For circle scenario:
```
roslaunch sensor_integration gps_imu_driver.launch gps_port:=/dev/ttyUSB1 gps_filename:=circle_gps imu_port:=/dev/ttyUSB0 imu_filename:=circle_imu
```
For town scenario:
```
roslaunch sensor_integration gps_imu_driver.launch gps_port:=/dev/ttyUSB1 gps_filename:=town_gps imu_port:=/dev/ttyUSB0 imu_filename:=town_imu
```

Step 3. Execute the script file that will do ros echo and also record bag file for each topic:
```
./echo_topics_and_record.sh circle
```

or 
```
./echo_topics_and_record.sh town
```
Step 4. Go to each pane in terminal and terminate the rosbag program.
```
Ctrl+B then Ctrl + C
```
To kill the tmux session:
```
tmux kill-session -t ros_session
```

Step 5. Create plots for gps and imu.
```
cd ../analysis
python3 gps.py
python3 deadReckoning.py
```