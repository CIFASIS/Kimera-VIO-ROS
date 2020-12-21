# Kimera-VIO-ROS

ROS Wrapper for [Kimera](https://github.com/MIT-SPARK/Kimera).

<div align="center">
    <img src="docs/media/Kimera-VIO-ROS_mesh.gif">
</div>

## Original repository
This repository is a modified version of [Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) (see original README below). We facilitate the installation process and the use of Docker.

## Docker support

In order to facilitate the installation process, the system is wrapped up using Docker.
We provide scripts to create a Docker image, build the system and run it in a Docker container. 

### Dependencies 
* Docker
* ROS
* [`pose_listener`](https://github.com/CIFASIS/pose_listener) (if you use `run_rosario_sequence.sh`, see below)

### Building the system
Run:
```
./run.sh -b
```

This command creates a Docker image, installs all the dependencies and builds the system. The resulting image contains a version of the system ready to be run.

### Running the system in VIS mode
If you are not interested in making changes in the source code, you should run the system in VIS mode. Run:
```
./run.sh -v
```
The system is launched in a Docker container based on the previously built image. By default, this command executes a launch file which is configured to run the Rosario dataset. If you want to run your own dataset, **write a launch file and placed it in** `launch/`. Configuration files must be placed in `Kimera-VIO/params/rosario_dataset/`. Then, run the script with the option `-l <LAUNCH_FILE_NAME>`. For example, if you are testing EuRoC, write `euroc_dataset.launch`, move it into `launch/` and type:
```
./run.sh -v -l euroc_dataset.launch
```
Making changes in launch/configuration files in the host is possible because these folders are mounted into the Docker container. It is not necessary to access the container through a bash shell to modify these files.

See below for information about input data and visualization.

### Running the system in DEV mode
DEV mode allows developers to make changes in the source code, recompile the system and run it with the modifications. To do this, the whole repository is mounted in a container. Run:
```
./run.sh -d
```
This opens a bash shell in a docker container. You can edit source files in the host and after that you can use this shell to recompile the system. When the compilation process finishes, you can run the method using `roslaunch`.

See below for information about input data and visualization.

### Input data and visualization

At this point, the system is waiting for input data. Either you can run `rosbag play` or you can use `run_rosario_sequence.sh`.
If you choose the latter, open a second terminal and run:
```
./run_rosario_sequence.sh -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
In contrast to what `run.sh` does, `run_rosario_sequence.sh` executes commands in the host (you can modify it to use a Docker container). 

`ROSBAG_FILE` is played using `rosbag`. Also, make sure you have cloned and built `pose_listener` in your catkin workspace. Default path for the workspace is `${HOME}/catkin_ws`, set `CATKIN_WS_DIR` if the workspace is somewhere else (e.g.: `export CATKIN_WS_DIR=$HOME/foo_catkin_ws`). `pose_listener` saves the estimated trajectory in `<OUTPUT_TRAJECTORY_FILE>` (use absolute path). You can edit `run_rosario_sequence.sh` if you prefer to save the trajectory using your own methods. Additionally, `run_rosario_sequence.sh` launches `rviz` to display visual information during the execution of the system.

Alternatively, if you are not interested in development but in testing or visualization, instead of running `run.sh` and `run_rosario_sequence.sh` in two different terminals, you can just run:
```
./run_rosario_sequence.sh -r -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
This launches a Docker container and executes the default launch file (see `LAUNCH_FILE` in `run.sh`). After that, the bagfile is played and `rviz` and `pose_listener` are launched. Add `-b` if you want to turn off the visualization.


# Original README

## Publications

We kindly ask to cite our paper if you find this library useful:

- A. Rosinol, M. Abate, Y. Chang, L. Carlone, [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/abs/1910.02490). IEEE Intl. Conf. on Robotics and Automation (ICRA), 2020. [arXiv:1910.02490](https://arxiv.org/abs/1910.02490).
 
 ```bibtex
 @InProceedings{Rosinol20icra-Kimera,
   title = {Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping},
   author = {Rosinol, Antoni and Abate, Marcus and Chang, Yun and Carlone, Luca},
   year = {2020},
   booktitle = {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
   url = {https://github.com/MIT-SPARK/Kimera},
   pdf = {https://arxiv.org/pdf/1910.02490.pdf}
 }
```

# 1. Installation

## A. Prerequisities

- Install ROS by following [our reference](./docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

- ROS non-default dependencies for [mesh_rviz_plugins](https://github.com/ToniRV/mesh_rviz_plugins) (change `melodic` for your ROS distribution):
```bash
sudo apt-get install ros-melodic-image-geometry ros-melodic-pcl-ros ros-melodic-cv-bridge
```

- System dependencies:
First, update package list: `sudo apt-get update`
```bash
sudo apt-get install -y --no-install-recommends apt-utils
sudo apt-get install -y \
      cmake build-essential unzip pkg-config autoconf \
      libboost-all-dev \
      libjpeg-dev libpng-dev libtiff-dev \
# Use libvtk5-dev, libgtk2.0-dev in ubuntu 16.04 \
      libvtk6-dev libgtk-3-dev \
      libatlas-base-dev gfortran \
      libparmetis-dev \
      python-wstool python-catkin-tools \
```

- GTSAM's Optional dependencies (highly recommended for speed)
Install [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/): `sudo apt-get install libtbb-dev`

## B. KimeraVIO ROS wrapper Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# On Ubuntu 16.04:
# catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON
catkin config --merge-devel

# Add workspace to bashrc for automatic sourcing of workspace.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
# For ssh:
git clone git@github.com:MIT-SPARK/Kimera-VIO-ROS.git
# For https:
# git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git

# Install dependencies from rosinstall file using wstool
wstool init # Use unless wstool is already initialized

# For ssh:
wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_ssh.rosinstall
# For https
# wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall

# Finally, download and update repos:
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/catkin_ws/devel/setup.bash
```

# 2. Usage
Download a [Euroc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) rosbag: for example [V1_01_easy](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag).

## Online
  1. As a general good practice, open a new terminal and run: `roscore`

  2. In another terminal, launch KimeraVIO ROS wrapper:
  ```bash
  roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch
  ```

  3. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_euroc.rviz
  ```
  > Note: this rviz configuration makes use of a rviz plugin: [mesh_rviz_plugins](https://github.com/ToniRV/mesh_rviz_plugins). To visualize the textured 3D mesh, clone this plugin to your catkin workspace and catkin build it (note that this should be done automatically via `wstool`).

  4. Finally, in another terminal, launch the downloaded Euroc rosbag:
  ```bash
  rosbag play --clock /PATH/TO/EUROC_ROSBAG
  ```

  > Note that you will need to both source ROS and your `catkin_ws` for each new terminal unless you added the following lines to your `~/.bashrc` file:
  > ```bash
  > source /opt/ros/melodic/setup.bash  # Change `melodic` for your ROS distribution.
  > source ~/catkin_ws/devel/setup.bash # Change `bash` to the shell you use.
  > ```

## Offline
  In this mode, the provided rosbag will be first parsed and then sent to the VIO for processing.
  This is particularly useful when debugging to avoid potential ROS networking issues.
  - To run, launch the KimeraVIO ROS wrapper with the `online` parameter set to `false` and specify the rosbag's path:
  ```bash
  roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=false rosbag_path:="PATH/TO/ROSBAG"
  ```

## Running Unit tests

To run unit tests using catkin for this specific package, call (after building the package and sourcing the workspace):

```bash
catkin run_tests --no-deps --this
```

## Other functionalities

### Using camera_info topics instead of Yaml parameters

It is sometimes convenient to use the `camera_info` topics to parse the camera's parameters.
There are currently two ways of using these topics:
 - Offline: using the launch file `launch/cam_info_yamlizer.launch` which will generate yaml files out of the topics.
 You need to make sure that the `frame_id`s and the ROS topics are correctly set. Also, mind that the left/right cam frame ids are typically set as static tfs in a rosbag, therefore, first launch the node, and then run the rosbag (in case you see an exception bcs of a missing frame_id).
 - Online: setting the flag `use_online_cam_params` (see `launch/kimera_vio_ros.launch`) to true, and ensuring ROS topics are correctly set.

### Restart Kimera-VIO

The typical use case is that you have multiple rosbags and you don't want to be killing Kimera-VIO(-ROS) each time.
If this is your case, then we provide a rosservice to restart Kimera-VIO (it will do a hard restart, meaning the whole pipeline and data provider will be destructed and constructed again).
```bash
rosservice call /kimera_vio_ros/kimera_vio_ros_node/restart_kimera_vio
```
> Note that Kimera-VIO will complain if timestamps are not strictly increasing. Therefore, one must follow these steps:
> 1. Start Kimera-VIO and rosbag
> 2. Stop rosbag
> 3. Call rosservice to restart VIO
> 4. Start another rosbag

### Enable Dense Depth Stereo estimation

This will run OpenCV's StereoBM algorithm, more info can be found [here](http://wiki.ros.org/stereo_image_proc) (also checkout this to [choose good parameters](http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters)):

```bash
roslaunch kimera_vio_ros kimera_vio_ros_euroc run_stereo_dense:=1
```

This will publish a `/stereo_gray/points2` topic, which you can visualize in Rviz as a 3D pointcloud.
Alternatively, if you want to visualize the depth image, since Rviz does not provide a plugin to
visualize a [disparity image](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html), we also run a [disparity_image_proc](https://github.com/ToniRV/disparity_image_proc) nodelet that will publish the depth image to `/stereo_gray/disparity_image_proc/depth/image_raw`.

# Hardware use

See the [documentation on hardware setup](docs/hardware_setup.md) for instructions on running KimeraROS on supported hardware platforms, as well as guides on how to develop for other platforms.

# BSD License
KimeraVIO ROS wrapper is open source under the BSD license, see the [LICENSE.BSD](./LICENSE.BSD) file.

