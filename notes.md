# Notes for multi robot SLAM simulation with Jackal Robots

## Swarm-SLAM

- Skipping this for now:
    If you are using lidar, install teaser++ with python bindings: https://teaser.readthedocs.io/en/latest/installation.html#installing-python-bindings
- no module named em and subsequent errors like AttributeError: module 'em' has no attribute 'BUFFERED_OPT':
     pip3 install empy==3.3.4

### Docker installation 
*Skip these steps if you have access to the repository on docker hub*
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

## Control of Jackal Robots in Simulation
[Jackal with ROS2 and Isaac Sim](https://forums.developer.nvidia.com/t/how-to-drive-clearpath-jackal-via-ros2-messages-in-isaac-sim/275907)

---
Good information about control of robots in Gazebo using ros2_control [here](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts/).

An example launch file for ros2_control is [here](https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/bringup/launch/diffbot.launch.py).

Installing teleop_twist_keyboard:
```bash
apt-get install ros-humble-teleop-twist-keyboard
```
using keyboard teleop, make sure to set the topic and stamped parameters correctly:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=my_cmd_vel -p stamped:=True 
```
---
# Starting from the container mylad13/humble-ros2controllers
### mylad13/jackal_multi_robot repo and mylad13/jackal repo
```bash
sudo docker run -itd --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name <container name> <image name>
```
and then
```bash
sudo docker exec -it <container name> bash
```
[Guide to running GUI apps in linux docker container from windows host](https://medium.com/@potatowagon/how-to-use-gui-apps-in-linux-docker-container-from-windows-host-485d3e1c64a3).

*This step can be skipped, it's already done: ros-humble-ros2-controllers and ros-humble-ros2-control are installed from binary, and the humble branch of gz_ros2_control is installed from source*. If starting from a container that does not have ros2_ws:
```bash
mkdir -p ~/ros2_ws/src
```
Then, install [ros2_controllers for humble](https://control.ros.org/humble/doc/getting_started/getting_started.html), and clone the repos into ~/ros2_ws/src
```bash
git clone https://github.com/mylad13/jackal_multi_robot
git clone https://github.com/mylad13/jackal -b humble-devel
```
 then go back into ros2_ws and run
```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
or 
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```
You don't need to execute colcon build every time you change your python code if you include the option --symlink-install.

---

Errors with gzserver not closing correctly might be fixed by
```bash
killall gzclient && killall gzserver
```
if killall is not there, install it by
```bash 
sudo apt-get update
sudo apt-get install psmisc
```
---
Facing errors when loading "gazebo_ros2_control":
``` [gzserver-1] [INFO] [1731356611.586901413] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin ```
We need to replace the gazebo_ros2_control plugin with ign_ros2_control plugin (for humble), which is compatible with ignition gazebo fortress...[documentation here](https://github.com/ros-controls/gz_ros2_control/blob/humble/doc/index.rst)

Official instructions from clearpath robotics for [simulation, localization, and navigation of their robots](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/nav2). This is still not multi-robot, but their packages may be used as inspiration for building our jackal_multi_robot package. Similar ideas already exist in the jackal_navigation package though.

---

- Dev Containers extension for VSCode may be helpful in working with docker containers.

---
Useful information on namespaces, node names, and tf2 mapping for simulating multiple robots [here](https://www.learnros2.com/ros/tutorials/multiple-robotic-arms-simulation-demo).