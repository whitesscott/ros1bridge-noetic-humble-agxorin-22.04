#!/usr/bin/env bash
set -euo pipefail

IMAGE=ros1_bridge:ros2            # adjust tag if needed
WORKDIR=/workspace

docker run -it --rm \
  --net=host --runtime=nvidia --privileged --ipc=host \
  -v "$HOME/.git/rosbridge":"$WORKDIR" \
  --workdir "$WORKDIR" \
  --name ros_bridge_example1B \
  $IMAGE bash -c "$(cat <<'EOF'
# -------- inside container ----------------------------------------------
#  1) Create a temporary tmux session
SESSION=bridge
tmux new-session  -d -s "$SESSION" -n A

#  2) Pane A  – ROS 1 master
tmux send-keys   -t "$SESSION":A \
  "source \$ROS1_INSTALL_PATH/setup.bash && roscore" C-m

#  3) Pane B  – bridge
tmux split-window -v -t "$SESSION":A
tmux select-pane  -t "$SESSION":A.1 -T B
tmux send-keys    -t "$SESSION":A.1 \
  "source \$ROS2_INSTALL_PATH/setup.bash && \
   source \$BRIDGE_LOCAL_SETUP_BASH && \
   export ROS_MASTER_URI=http://localhost:11311 && \
   ros2 run ros1_bridge dynamic_bridge" C-m

#  4) Pane C  – ROS 2 talker
tmux split-window -h -t "$SESSION":A
tmux select-pane  -t "$SESSION":A.2 -T C
tmux send-keys    -t "$SESSION":A.2 \
  "source \$ROS2_INSTALL_PATH/setup.bash && ros2 run demo_nodes_py talker" C-m

#  5) Pane D  – ROS 1 listener
tmux split-window -h -t "$SESSION":A.1   # split pane B horizontally
tmux select-pane  -t "$SESSION":A.3 -T D
tmux send-keys    -t "$SESSION":A.3 \
  "source \$ROS1_INSTALL_PATH/setup.bash && rosrun roscpp_tutorials listener" C-m

tmux select-layout -t "$SESSION": tiled
tmux attach -t "$SESSION"
EOF
)"

