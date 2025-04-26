ros1_bridge Dockerfile to build one 15gb docker image with both ROS1/noetic and ROS2/humble on nvcr.io/nvidia/l4t-jetpack:r36.4.0.

It runs on Jetson AGX Orin 32gb dev kit that is running Jetpack 6.2 / 36.4.3

Build the docker image with this command.

docker build -t ros1_bridge:ros2 .


https://github.com/ros2/ros1_bridge/blob/master/README.md#example-1a-ros-1-talker-and-ros-2-listener

tmux is installed in image. Here are 4 tmux terminal based shell scripts that execute the ros1_bridge examples 1a, 1b, 3, and 4. didn’t do #2 as am running AGX via ssh.

ros1talk_ros2hear.sh
ros2talk_ros1hear.sh
ex3_AddTwoInts.sh
ex4_AddTwoInts.sh



ros1bridge.sh is a bash script to start the container. Please edit before use to change the volume mount to what makes sense in your environment.

docker run -it --rm --net=host --runtime nvidia --privileged --ipc=host -v $HOME/.git/rosbridge:/workspace ros1_bridge:ros2 bash




PS: The tmux pane that executes “ros2 run ros1_bridge” emits this warning that is benign and just annoying: “failed to create 2to1 bridge for 
                                 topic ‘/rosout’ with ROS 2 type ‘rcl_interfaces/msg/Log’ and ROS 1 type 'rosgraph_msgs/Log’”
