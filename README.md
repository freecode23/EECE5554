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

9. To list rosservice listed from turtle sime node first run 
```
rosrun turtlesim turtlesim_node
```

8.2 Make a request
```
rosservice call /add_two_ints "a:2 b: 5"
```