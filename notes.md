# Notes for multi robot SLAM simulation with Jackal Robots

## Swarm-SLAM

- Skipping this for now:
    If you are using lidar, install teaser++ with python bindings: https://teaser.readthedocs.io/en/latest/installation.html#installing-python-bindings
- no module named em and subsequent errors like AttributeError: module 'em' has no attribute 'BUFFERED_OPT':
     pip3 install empy==3.3.4

### Docker installation
Following their instructions, I built the docker container. Simply running these commands leads to error, because some requirements are not met?
```bash 
git clone https://github.com/lajoiepy/cslam_experiments.git
cd cslam_experiments/docker
make build
make gpu_run
make swarmslam-lidar
```

attempting to run the cslam_experiments launch file, we get errors. We have to install Zenoh now:
```bash
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update 
sudo apt install zenoh-bridge-ros2dds
```
seemed to work. Zenoh must be used the way it is used in cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py
also we have to downgrade numpy:
```bash
pip install "numpy<2.0"
```

The problem is now that the docker container does not have access to Xserver, and I cannot run rviz or any other visualization. It is possible to do it when building the image with:
```
version: "3"services:
  app:
    image: my-app:latest
    build: .
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    network_mode: host
```
or by running the container with the following command:
```bash
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  container_name /bin/bash
```
inside the package, I run:
```bash
apt-get install xauth 
apt-get install xorg
```
also I did:
```bash
apt install ros-humble-rviz2
```
This should enable you to launch the experiment_lidar.launch.py file.
Attempting to launch kitti_lidar.launch.py results in error because rtabmap is missing. Therefore:
```bash
sudo apt install ros-humble-rtabmap*
```

Finally, I commited this container into a new image called x11swarmslam, which can be used to run containers with the above changes implemented. Run this command:
```bash
sudo docker run -itd --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name x11swarmslam x11swarmslam
```
and then
```bash
sudo docker exec -it x11swarmslam bash
```

----

In order to visualize the map using rerun, it is suggested to use docker. To avoid running a docker container as a child of another docker container, read [this post](https://stackoverflow.com/questions/27879713/is-it-ok-to-run-docker-from-inside-docker)

----
An alternative to RTAB-MAP is [FAST_LIO](https://github.com/hku-mars/FAST_LIO). Example of how to use FAST_LIO on the GrAco dataset is [given here](https://github.com/MISTLab/Swarm-SLAM/issues/49#issuecomment-2339632442).

----
Help with running the datasets:
https://github.com/MISTLab/Swarm-SLAM/issues/2


----
[Debugging Instructions for ROS2 Nodes](https://gist.github.com/JADC362/a4425c2d05cdaadaaa71b697b674425f)

## Jackal
[Jackal with ROS2 and Isaac Sim](https://forums.developer.nvidia.com/t/how-to-drive-clearpath-jackal-via-ros2-messages-in-isaac-sim/275907)

### Installing Jackal Desktop Software (from source)
[Instructions](https://clearpathrobotics.com/assets/guides/foxy/jackal/JackalInstallDesktopSoftware.html)
I made the jackal_ws directory in /
and added source /jackal_ws/install/setup.bash to ~/.bashrc

I commited this container to **x11swarmjackal**

Attempting to launch jackal_gazebo jackal_world.launch.py leads to this error:
``` executable 'spawner.py' not found on the libexec directory '/opt/ros/humble/lib/controller_manager'  ```
so I installed gazebo for ros humble (edit: not the best approach, refer to a few lines down) 
```bash
sudo apt-get install ros-humble-ros-gz
```
That did not fix the error. The problem seems to be because spawner does not have the .py extension in humble. Let's try to navigate to the launch file that is trying to run the spawner.
<!-- First install colcon-cd to enable navigation to ros2 folder directories: (edit: not used)
```bash
sudo apt install python3-colcon-cd
source /usr/share/colcon_cd/function/colcon_cd.sh
``` -->
At first I thought the launch file is in the Gazebo package. We cannot navigate to the package, because it was built from binary. Let's circle back and [build Gazebo Fortress from source](https://gazebosim.org/docs/fortress/install_ubuntu_src/). I exited and stopped the docker container, and ran it again from the x11swarmjackal image that DOES not have gazebo installed yet.
Following the gazebo installation from source, I added the source setup.bash of this workspace to ~/.bashrc:
```bash
source /workspace/install/setup.bash
```
It turns out that we have to edit /jackal_ws/src/jackal/jackal_control/launch/control.launch.py to change instances of ``` spawner.py ``` to ``` spawner ```. Problem solved! (so installing gazebo from source was not required... at least so far!)

so... launching jackal with
```bash
ros2 launch jackal_gazebo jackal_world.launch.py
```
is successful, with some errors. First error is ``` [gzserver-1] [Err] [OpenAL.cc:84] Unable to open audio device[default] 
[gzserver-1]  Audio will be disabled. ```
Second error is ``` [gzserver-1] [Err] [Sensor.cc:510] Get noise index not valid ``` with discussions on a possible fix [here](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/799). And I am not sure about the following:
```
[gzserver-1] [rcutils|error_handling.c:65] an error string (message, file name, or formatted message) will be truncated
[gzserver-1] [ERROR] [1730926730.377621476] [gazebo_ros2_control]: parser error Couldn't parse parameter override rule: '--param robot_description:=<?xml version="1.0" ?>
[gzserver-1] <!-- =================================================================================== -->
[gzserver-1] <!-- |    This document was autogenerated by xacro from /jackal_ws/install/jackal_description/share/jackal_description/urdf/jackal.urdf.xacro | -->
[gzserver-1] <!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
[gzserver-1] <!-- =================================================================================== -->
[gzserver-1] <robot name="jackal">
[gzserver-1]   <material name="dark_grey">
[gzserver-1]     <color rgba="0.2 0.2 0.2 1.0"/>
[gzserver-1]   </material>
[gzserver-1]   <material name="light_grey">
[gzserver-1]     <color rgba="0.4 0.4 0.4 1.0"/>
[gzserver-1]   </material>
[gzserver-1]   <material name="yellow">
[gzserver-1]     <color rgba="0.8 0.8 0.0 1.0"/>
[gzserver-1]   <, at ./src/rcl/arguments.c:343
[gzserver-1] 
[imu_filter_madgwick_node-6] [INFO] [1730926730.387032402] [imu_filter_node]: First IMU message received.
[INFO] [spawn_entity.py-4]: process has finished cleanly [pid 322106]
```
followed by these warnings:
```
[spawner-8] [WARN] [1730926739.032171794] [spawner_jackal_velocity_controller]: Could not contact service /controller_manager/list_controllers
[spawner-8] [INFO] [1730926739.033102827] [spawner_jackal_velocity_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-7] [WARN] [1730926739.035220524] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-7] [INFO] [1730926739.036111111] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
```
I am committing this container once again to **x11swarmjackal**

---
## jackal_multi_robot package

clone the repo into /jackal_ws/src, then go back into jackal_ws and run
```bash
rosdep install --from-paths src -r -y
colcon build --symlink-install
source ~/.bashrc
```
You don't need to execute colcon build every time you change your python code if you include the option --symlink-install.

Errors with gzserver might be fixed by
```bash
killall gzclient && killall gzserver
```
if killall is not there, install it by
```bash 
sudo apt-get update
sudo apt-get install psmisc
```
Facing errors when loading "gazebo_ros2_control":
``` [gzserver-1] [INFO] [1731356611.586901413] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin ```
I think we need to replace the gazebo_ros2_control plugin with gz_ros2_control (or ign_ros2_control for humble) plugin, which is compatible with non-classic versions of gazebo...[documentation here](https://github.com/ros-controls/gz_ros2_control/blob/humble/doc/index.rst)
I did some research regarding navigation of Jackals in ROS2 humble, and came upon these official instructions from clearpath robotics for [simulation, localization, and navigation of their robots](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/nav2). This is still not multi-robot, but their packages may be used as inspiration for building our jackal_multi_robot package. Similar ideas already exist in jackal_navigation package though.

---
Good information about control of robots in Gazebo using ros2_control [here](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts/).
IMPORTANT: Install [ros2_controllers for humble](https://control.ros.org/humble/doc/getting_started/getting_started.html)
---

- Dev Containers extension for VSCode may be helpful in working with docker containers.