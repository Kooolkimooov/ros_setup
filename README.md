# ROS installation and setup for BlueROVs

Tested with:
- Ubuntu 18.04.6 LTS with ROS Melodic 1.14.13 paired with Brice on 23/05/2024
- Ubuntu 20.04.6 LTS with ROS Noetic 1.16.0 **NOT TESTED WITH ACTUAL ROV**

## ROS installation

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
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-catkin-tools build-essential -y # if you are on Ubuntu 18
```
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential -y # if you are on Ubuntu 20
```
5. initialize and update `rosdep`: 
```bash
sudo rosdep init && rosdep update
```

## Catkin setup

With a new terminal (this is to allow the `setup.bash` to be sourced).

1. create a catkin directory: 
```bash
mkdir -p catkin/src && cd catkin
```
2. build and add catkin to `.bashrc`:
```bash
catkin build && sed -i '2i source ~/catkin/devel/setup.bash' ~/.bashrc
``` 
3. source your `~/.bashrc`.
4. install mavros dependencies: 
```bash
sudo apt install ros-melodic-geographic-msgs libgeographic-dev -y # if you are on Ubuntu 18
```
```bash
sudo apt install ros-noetic-geographic-msgs libgeographic-dev -y # if you are on Ubuntu 20
```

For COSMER blueROVs (as of 23/05/2024) we need to make modification to the mavros code because the on-board version of ROS requires it, thus we **install mavros and mavlink from source**.

5. install mavros and mavlink:
```bash
wstool init src &&
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall &&
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall &&
wstool merge -t src /tmp/mavros.rosinstall &&
wstool update -t src &&
catkin build
```
If the build fails you may need to install the future python library: `pip install future`

6. now the code modification:
   - at `mavros/mavros_msgs/srv/SetMode.srv`, line 22 replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp`, line 1200 (melodic) 1224 (noetic) replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp`, line 1220 (melodic) 1244 (noetic) replace `mode_sent` with `success` 
   - at `mavros/mavros_msgs/msg/OverrideRCIn.msg`, line 9 replace `uint16[18]` with `uint16[8]` 
7. rebuild: 
```bash
catkin build
```
If you **do not need to modify the source code**, install with:
```bash
sudo apt install ros-melodic-mavros # if you are on Ubuntu 18
```
```bash
sudo apt install ros-noetic-mavros # if you are on Ubuntu 2O
```

## Network setup

COSMER's BlueROVs operate on the `192.168.254.x` network using the "blue box" usb network interface. The computer with the ROS Core is expected to be at `192.168.254.15` and the BlueROVs are on other adresses: Brice is on `192.168.254.16` for example.

We now need to setup the network **with the blue box plugged in**:

1. navigate to `settings > Network` in the GUI
2. click on the cog for the `USB Ethernet` device
3. in the `IPv4` tab, set the `IPv4 Method` to `Manual`
4. fill in the `Address` as `192.168.254.15`
5. fill in the `Netmask` as `255.255.255.0`
6. `Apply`

ROS also needs to use the blue box:
```bash
sed -i '1i export ROS_IP=192.168.254.15' ~/.bashrc &&
sed -i '1i export ROS_HOSTNAME=192.168.254.15' ~/.bashrc &&
sed -i '1i export ROS_MASTER_URI=http://192.168.254.15:11311' ~/.bashrc
```
Please note that **you may need to remove these lines from** `~/.bashrc` if you use ROS for other applications.

## Actual use of the BlueROV

To begin using the blueROV we need a way to control it, the simplest of which is a controller. You can clone this basic package:
```bash
cd ~/catkin/src && git clone https://github.com/Kooolkimooov/bluerov
```
and build it:
```bash
cd ~/catkin && catkin build
```
This package is compatible with the Logitech F310 controllers commonly used at COSMER. If you do not have this controller you may need to modify the code.

To use the blueROV, please follow these instructions:

1. ensure that the blue box is properly connected to the computer and that the network settings discussed in [the network section](#network-setup)
2. ensure that the BlueROV is properly connected to the blue box
3. launch the ROS core:
```bash
roscore
```
4. turn on the BlueROV, **ensure that the battery compartment is properly sealed before submerging it**
5. wait for the BlueROV to properly boot up, you can check the topic list to have an hint of the progress:
```bash
rostopic list
```
6. ensure that your controller is properly plugged in, then run:
```bash
roslaunch bluerov simple_controller.launch
```
7. the controller is disarmed by default, to arm the controller press the `A` button; you can now control the BlueROV with the controller; to disarm the controller use the `B` button.

The controller mapping can be changed with:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```


