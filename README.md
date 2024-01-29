# Labs and Projects associated with EECE5554-Robot Sensing and Navigation

## Shortcut

1. To start ros master:
roscore

2. To start ros node:
```
rosrun rospy_tutorial talker
```

3. To show the graph of all nodes:
```
rqt_graph
```

4. To list all nodes:
```
rosnode list
```

5. To kill node using rosnode:
```
rosnode kill /<node_name>
```

6. To list topic:
```
rostopic list
```
It will list the type of topic. i.e. std_msgs/String,
the publisher and subsciber nodes.

7. To see what is published in the node:
```
rostopic echo /turtle1/pose
```

8. To see ros service
```
rosservice list
```
8.1 Run rosservice
```
rosrun rospy_tutorials add_tow_ints_server
```

8.2 To list rosservice listed from turtle sime node first run 
```
rosrun turtlesim turtlesim_node
```

8.3 Make a request
```
rosservice call /add_two_ints "a:2 b: 5"
```

9. To check if a package exist:
```
rosmsg show gps_driver
```

# 2. Catkin setup:

Create the workspace:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
```

Compile the workspace to create build and devel:
```
catkin_make
```

Source the setup bash to get the ROS environment:
```
source ~/Desktop/code/EECE5554/lab1/catkin_ws/devel/setup.bash
gedit -/.bashrc
```

Add the workspace setup bash to .bashrc ensure that your ROS environment is automatically set up every time you open a new terminal window.


# 3. Create package:
Package represent the module of the robot. It could be your gps reader, motor driver, etc.
Inside the catkin_ws/src directory:
```
cd src/
catkin_create_pkg <package_name> <dependencies separated by space>
```

Example: 
catkin_create_pkg gps_driver message_generation message_runtime std_msgs rospy

Now we can put our source file inside the src file of this new package.

# 4. To add custom message:

1. Add you message structure iniside your package_name/msg/my_msg.msg
2. Modify CMAKEList.txt and package.xml file:
In CMakeLists.txt, make sure the following exist and uncommented:

1)
```
find_package(catkin REQUIRED
             COMPONENTS
             message_generation
             std_msgs)
```

2)
```
add_message_files(
  FILES
  CustomGPS.msg
)
```

3)
```
generate_messages(
  DEPENDENCIES 
  std_msgs)
```

In package.xml, make sure you have:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

3. In catkin_ws, run: `catkin_make`
Then update the environment:
```
source ./devel/setup.bash
```

4. Check if the package is created:
```
rospack find gps_driver 
```


