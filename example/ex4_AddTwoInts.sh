#!/usr/bin/env bash

SESSION="ros_bridge_param"

# 1) Create the bridge.yaml file
cat > bridge.yaml << 'EOF'
topics:
  -
    topic: /chatter  # Topic name on both ROS 1 and ROS 2
    type: std_msgs/msg/String  # Type of topic to bridge
    queue_size: 1  # Queue size
services_2_to_1:
  -
    service: /add_two_ints  # ROS 1 service name
    type: roscpp_tutorials/TwoInts  # The ROS 1 service type name
EOF

# 2) Launch tmux session if not already running
if ! tmux has-session -t $SESSION 2>/dev/null; then
  tmux new-session -d -s $SESSION

  # Split into 3 vertical columns
  tmux split-window -h -t $SESSION:0
  tmux split-window -h -t $SESSION:0

  # Now split each column into top/bottom → total 6 panes
  tmux split-window -v -t $SESSION:0.0
  tmux split-window -v -t $SESSION:0.1
  tmux split-window -v -t $SESSION:0.2

  # Tidy up into an even 3×2 grid
  tmux select-layout -t $SESSION:0 tiled

  # Pane 0 (top-left): ROS1 roscore
  tmux send-keys -t $SESSION:0.0 "source \$ROS1_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.0 "roscore" C-m

  # Pane 1 (top-middle): load bridge.yaml & talker
  tmux send-keys -t $SESSION:0.1 "source \$ROS1_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.1 "rosparam load bridge.yaml" C-m
  tmux send-keys -t $SESSION:0.1 "rosrun rospy_tutorials talker" C-m

  # Pane 2 (top-right): add_two_ints_server
  tmux send-keys -t $SESSION:0.2 "source \$ROS1_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.2 "rosrun roscpp_tutorials add_two_ints_server" C-m

  # Pane 3 (bottom-left): parameter_bridge (ROS 1 ↔ ROS 2)
  tmux send-keys -t $SESSION:0.3 "source \$ROS2_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.3 "source \$BRIDGE_LOCAL_SETUP_BASH" C-m
  tmux send-keys -t $SESSION:0.3 "export LD_LIBRARY_PATH=/opt/ros/noetic/lib:\$LD_LIBRARY_PATH:/usr/local/cuda/lib64" C-m
  tmux send-keys -t $SESSION:0.3 "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

  # Pane 4 (bottom-middle): ROS 2 listener
  tmux send-keys -t $SESSION:0.4 "source \$ROS2_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.4 "ros2 run demo_nodes_cpp listener" C-m

  # Pane 5 (bottom-right): call the add_two_ints service
  tmux send-keys -t $SESSION:0.5 "source \$ROS2_INSTALL_PATH/setup.bash" C-m
  tmux send-keys -t $SESSION:0.5 "ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 1, b: 2}\"" C-m
fi

# 3) Attach to the session
tmux attach-session -t $SESSION
