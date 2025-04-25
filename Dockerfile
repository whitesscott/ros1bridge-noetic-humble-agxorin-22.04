################################  Docker build stage 1
FROM ros:noetic-robot-focal AS ros1

# install ros packages
RUN mkdir -p /tmp && chmod 1777 /tmp && \
    apt-get update && \
    apt-get install -y --no-install-recommends libyaml-cpp0.6 ros-noetic-desktop-full=1.5.0-1* wget \
        ros-noetic-common-tutorials ros-noetic-visualization-tutorials ros-noetic-urdf-tutorial && \
    apt-get install -y --no-install-recommends --reinstall \
        libboost-program-options1.71.0 libboost-chrono1.71.0 && \
    rm -rf /var/lib/apt/lists/*

#prep to copy 20.04 libraries required on 22.04 for ros1_bridge
RUN wget http://ports.ubuntu.com/pool/universe/p/poco/libpocofoundation62_1.9.2-3ubuntu3_arm64.deb && \
    apt-get install -y ./libpocofoundation62_1.9.2-3ubuntu3_arm64.deb && \
    rm ./libpocofoundation62_1.9.2-3ubuntu3_arm64.deb && \
    rm -rf /var/lib/apt/lists/*

RUN cd /usr/lib/aarch64-linux-gnu && \
    tar --create --file=/tmp/libraries.tar \
        libboost* \
        libicudata.so.66* \
        libicuio.so.66* \
        libicuuc.so.66* \
        libtinyxml2.so.6* \
        libyaml-cpp.so.0.6* \
        libicutest.so.66* \
        libbz2.so.1.0* \
        libicui18n.so.66* \
        libicutu.so.66* \
        liblog4cxx.* \
        libyaml-0.so.2* \
        libconsole_bridge.so.0.4* \
        libsqlite3.so.0* \
        libPoco*

################################  Docker build stage 2
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# — copy ROS 1 tree and libs required to build ros1_bridge —
COPY --from=ros1 /opt/ros/noetic /opt/ros/noetic
RUN mkdir -p /tmp && chmod 1777 /tmp
COPY --from=ros1 /tmp/libraries.tar /tmp/libraries.tar
COPY --from=ros1 /usr/include/boost /usr/include/boost

# Extract libraries for ros1_bridge compilation
RUN tar -xf /tmp/libraries.tar -C /usr/lib/aarch64-linux-gnu && \
    rm /tmp/libraries.tar

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ROS Noetic and ros1_bridge require python3.8 bits on 22.04
RUN apt-get update && \
    apt-get install -y software-properties-common curl && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8 libpython3.8 libpython3.8-minimal && \
    apt-get upgrade -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Setup apt for ROS2 Humble installation.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# — install build tools including python packages —
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git gnupg lsb-release \
    python3-pip wget python3-flake8 python3-flake8-blind-except \
    python3-flake8-builtins python3-flake8-class-newline \
    python3-flake8-comprehensions python3-flake8-deprecated \
    python3-flake8-docstrings python3-flake8-import-order \
    python3-flake8-quotes python3-pip python3-pytest \
    python3-pytest-cov python3-pytest-repeat tmux nano && \
    rm -rf /var/lib/apt/lists/*

# More required pip packages
RUN python3 -m pip install -U \
      setuptools==70.0.0 empy==3.3.4 \
      colcon-common-extensions vcstool rosdep osrf_pycommon lark defusedxml

# locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

################################ install ROS 2 Humble, Tutorials and cyclonedds
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp cyclonedds-dev ros-humble-cyclonedds libcycloneddsidl0 \
    ros-humble-rmw-implementation ros-humble-rmw ros-humble-rmw-implementation-cmake \
    ros-humble-rmw-dds-common ros-humble-action-tutorials-cpp ros-humble-action-tutorials-interfaces \
    ros-humble-action-tutorials-py ros-humble-as2-rviz-plugins \
    ros-humble-geometry-tutorials ros-humble-py-trees-ros* \
    ros-humble-turtlebot4*tutorials ros-humble-urdf-tutorial liblog4cxx12 && \
    rm -rf /var/lib/apt/lists/*

#Learned that ros1_bridge/README.md docs are old and current Humble/22.04 apt/debs are just as good and faster/smaller to install than 'from source'.
#WORKDIR /opt/ros/humble
#RUN mkdir src
#RUN curl -sSL https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos -o ros2.repos && \
#    vcs import src < ros2.repos
#RUN apt-get update && apt-get upgrade -y

#following is run just to do it in case it's later needed
RUN rosdep init || true && rosdep update

#RUN rosdep install --from-paths src --ignore-src -y -r \
#      --rosdistro humble \
#      --skip-keys "python3-catkin-pkg-modules fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-math6 ignition-cmake2"
#RUN colcon build --symlink-install --packages-skip ros1_bridge

################################  Build ros1_bridge.
#sed to comment out apt/deb conflict between ros and ros2 catkin versions.
RUN mv /etc/apt/sources.list.d/ros2.list /root/ && \
    apt-get update -y && \
sed  -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status && apt-get install -f && \
    mkdir -p /debs && cd /debs && \
    apt-get download python3-catkin-pkg python3-rospkg python3-rosdistro && \
    dpkg --force-overwrite -i *.deb && apt-get install -f && \
    rm -rf /debs && \
    mv /root/ros2.list /etc/apt/sources.list.d/ && \
    apt-get update -y

#We copy ros-noetic-desktop-full=/opt/ros/noetic so will skip ros-core-dev as it removes too many ros2 packages.
#RUN apt-get install -y ros-core-dev && rm -rf /var/lib/apt/lists/*
    #apt-get remove -y python3-catkin-pkg-modules && \

ENV ROS1_INSTALL_PATH=/opt/ros/noetic ROS2_INSTALL_PATH=/opt/ros/humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH

RUN mkdir -p /opt/ros1_bridge/src && \
    cd /opt/ros1_bridge/src && \
    git clone https://github.com/ros2/ros1_bridge && \
    source /opt/ros/humble/setup.bash && \
    source /opt/ros/noetic/setup.bash && \
    colcon build --symlink-install --packages-select ros1_bridge

################################ Completed ros1_bridge install. Now do final stuff for docker image.
WORKDIR /

ENV ROS1_DISTRO=noetic ROS2_DISTRO=humble \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    BRIDGE_LOCAL_SETUP_BASH=/opt/ros1_bridge/src/install/local_setup.bash ROS_DISTRO=

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
