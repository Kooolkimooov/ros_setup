# ROS installation and setup for BlueROVs

Tested with:
- Ubuntu 18.04.6 LTS with ROS Melodic 1.14.13 paired with Brice on 23/05/2024
- Ubuntu 20.04.6 LTS with ROS Noetic 1.16.0 **NOT TESTED WITH ACTUAL ROV**

## ROS install

Following [this](http://wiki.ros.org/Installation/Ubuntu) tutorial.

1. add ros as an apt source:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && 
sudo apt update
```
2. install ros:
```bash
sudo apt install ros-melodic-desktop-full -y # if you are on Ubuntu 18
```   
```bash
sudo apt install ros-noetic-desktop-full -y # if you are on Ubuntu 20
``` 
2. add ros to `.bashrc`:
```bash
sed -i '1i source /opt/ros/melodic/setup.bash' ~/.bashrc # if you are on Ubuntu 18
``` 
```bash
sed -i '1i source /opt/ros/noetic/setup.bash' ~/.bashrc # if you are on Ubuntu 20
``` 
3. source your `~/.bashrc`.
4. install dependencies for building packages: 
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-catkin-tools build-essential - y # if you are on Ubuntu 18
```
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential -y # if you are on Ubuntu 20
```
5. initialize and update `rosdep`: 
```bash
sudo rosdep init && rosdep update
```

## Catkin setup

for COSMER blueROVs (as of 2024) we need to make modification to the mavros code because the on-board version of ROS requires it, thus we **install mavros and mavlink from source**.

1. create a catkin directory: 
```bash
mkdir -p catkin/src && cd catkin
```
2. build and add catkin to `.bashrc`:
```bash
catkin build && sed -i '1i source ~/catkin/devel/setup.bash' ~/.bashrc
``` 
3. source your `~/.bashrc`.
4. install mavros dependencies: 
```bash
sudo apt install ros-melodic-geographic-msgs libgeographic-dev -y # if you are on Ubuntu 18
```
```bash
sudo apt install ros-noetic-geographic-msgs libgeographic-dev -y # if you are on Ubuntu 20
```
5. install mavros and mavlink:
```bash
wstool init src &&
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall &&
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall &&
wstool merge -t src /tmp/mavros.rosinstall &&
wstool update -t src &&
catkin build
```
if the build fails you may need to install the future python library: `pip install future`

6. now the code modification:
   - at `mavros/mavros_msgs/srv/SetMode.srv`, line 22 replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp`, line 1200 (melodic) 1224 (noetic) replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp`, line 1220 (melodic) 1244 (noetic) replace `mode_sent` with `success` 
   - at `mavros/mavros_msgs/msg/OverrideRCIn.msg`, line 9 replace `uint16[18]` with `uint16[8]` 
7. rebuild: 
```bash
catkin build
```
   