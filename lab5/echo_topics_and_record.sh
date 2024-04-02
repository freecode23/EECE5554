#!/bin/bash

# Check for the scenario argument
if [ "$1" != "circle" ] && [ "$1" != "town" ]; then
  echo "Usage: $0 [circle|town]"
  exit 1
fi

scenario=$1

# Start a new tmux session without attaching, creating the first window for rostopic echo commands
tmux new-session -d -s ros_session -n data_streams

# Split the window into two panes horizontally for the rostopic echo commands
tmux split-window -h

# Select the first pane and run the rostopic echo for /imu
tmux select-pane -t 0
tmux send-keys "source ~/Desktop/code/EECE5554/integration_ws/devel/setup.bash; rostopic echo /imu" C-m

# Select the second pane and run the rostopic echo for /gps
tmux select-pane -t 1
tmux send-keys "source ~/Desktop/code/EECE5554/integration_ws/devel/setup.bash; rostopic echo /gps" C-m

# Split the first pane vertically for the rosbag record for /imu
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "cd ~/Desktop/code/EECE5554/integration_ws/data/; rosbag record -O ${scenario}_imu/${scenario}_imu.bag /imu" C-m

# Split the second pane vertically (which is now pane 2) for the rosbag record for /gps
tmux select-pane -t 2
tmux split-window -v
tmux send-keys "cd ~/Desktop/code/EECE5554/integration_ws/data/; rosbag record -O ${scenario}_gps/${scenario}_gps.bag /gps" C-m

# Attach to the session
tmux attach-session -t ros_session
