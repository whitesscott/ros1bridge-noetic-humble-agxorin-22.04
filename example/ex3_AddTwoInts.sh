#!/usr/bin/env bash

# Define the tmux session name
SESSION="ros_session"

# Only create if it doesn't already exist
if ! tmux has-session -t $SESSION 2>/dev/null; then
  echo "Creating new tmux session: $SESSION"
  tmux new-session -d -s $SESSION

  # 1) Split into left and right halves
  tmux split-window -h -t $SESSION:0

  # 2) Split each half into top and bottom
  tmux split-window -v -t $SESSION:0.0   # left half → top-left & bottom-left
  tmux split-window -v -t $SESSION:0.1   # right half → top-right & bottom-right

  # 3) Enforce a clean 2×2 grid
  tmux select-layout -t $SESSION:0 tiled

  # Pane numbering is now:
  #   0: top-left      1: top-right
  #   2: bottom-left   3: bottom-right

  # Send your ROS commands into each pane:

  # Pane 0 (top-left): ROS1 core
  tmux send-keys -t $SESSION:0.0 "source \$ROS1_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.0 "roscore -p 11311" C-m

  # Pane 1 (top-right): ROS1 AddTwoInts server
  tmux send-keys -t $SESSION:0.1 "source \$ROS1_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.1 "export ROS_MASTER_URI=http://localhost:11311" C-m
  tmux send-keys -t $SESSION:0.1 "rosrun roscpp_tutorials add_two_ints_server" C-m

  # Pane 2 (bottom-left): ROS2 AddTwoInts client:
  tmux send-keys -t $SESSION:0.2 "export LD_LIBRARY_PATH=/usr/local/cuda/lib64" C-m
  tmux send-keys -t $SESSION:0.2 "source \$ROS2_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.2 "ros2 run demo_nodes_cpp add_two_ints_client" C-m

  # Pane 3 (bottom-right): ROS1-ROS2 bridge
  tmux send-keys -t $SESSION:0.3 "source \$ROS2_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.3 "source \$BRIDGE_LOCAL_SETUP_BASH" C-m
  tmux send-keys -t $SESSION:0.3 "export LD_LIBRARY_PATH=/opt/ros/noetic/lib:\$LD_LIBRARY_PATH:/usr/local/cuda/lib64" C-m
  tmux send-keys -t $SESSION:0.3 "export ROS_MASTER_URI=http://localhost:11311" C-m
  tmux send-keys -t $SESSION:0.3 "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m
fi

echo "Attaching to tmux session: $SESSION"
tmux attach-session -t $SESSION
