# Define parent image
FROM ros:melodic-perception

# Set environment and working directory
ENV CATKIN_WS=/root/catkin_ws
WORKDIR $CATKIN_WS
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y \
    ros-melodic-image-geometry \
    ros-melodic-pcl-ros \
    ros-melodic-cv-bridge \
    ros-melodic-tf-conversions \
    ros-melodic-interactive-markers \
    ros-melodic-rviz unzip \
    libtool autoconf libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    python-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# Copy files
COPY ./ ./src/Kimera-VIO-ROS

# Build SLAM system
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin config --merge-devel && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    cd ./src && \
    wstool init && \
    wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall && \
    wstool update && \
    cd ../ && \
    catkin build

# Define CMD
CMD /bin/bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch kimera_vio_ros kimera_vio_ros_rosario.launch"
