FROM ros:melodic-perception

ENV CATKIN_WS=/root/catkin_ws \
    KIMERA_ROOT=/root/catkin_ws/src/Kimera-VIO-ROS \
    DEBIAN_FRONTEND=noninteractive

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

COPY . $KIMERA_ROOT
COPY ./scripts $CATKIN_WS

WORKDIR $CATKIN_WS

RUN /bin/bash -c "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./build.sh && ./modify_entrypoint.sh"

#CMD /bin/bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch kimera_vio_ros kimera_vio_ros_rosario.launch"
