# ROS installation and setup for BlueROVs

Tested with:
- Ubuntu 18.04.6 LTS with ROS Melodic 1.14.13 paired with Brice on 23/05/2024

## ROS install

Following [this](http://wiki.ros.org/Installation/Ubuntu) tutorial.

1. run `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. install curl if necessary: `sudo apt install curl`
3. run `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
4. run `sudo apt update`
5. run `sudo apt install ros-melodic-desktop-full`
6. add either:
   - `source /opt/ros/melodic/setup.bash` to `~/.bashrc`
   - `source /opt/ros/melodic/setup.zsh` to `~/.zshrc`
7. source your `~/.bashrc` or `~/.zshrc`
8. install dependencies for building packages: `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
9. initialize and update `rosdep`: `sudo rosdep init && rosdep update`

## Catkin setup

for COSMER blueROVs (as of 2024) we need to make modification to the mavros code because the on-board verstion of ROS requires it, thus we install mavros and mavlink from source.

1. create a catkin directory: `mkdir -p catkin/src` and go into that directory: `cd catkin`
2. run `wstool init src`
3. install mavros and mavlink:
   - `rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall`
   - `rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall`
   - `wstool merge -t src /tmp/mavros.rosinstall`
   - `wstool update -t src`
   - `wstool update`
4. once that is done you can do a build: `catkin build` if the build fails you may need to install pip `sudo apt install python-pip` and the future python library `pip install future`
5. now the code modification:
   - at `mavros/mavros_msgs/srv/SetMode.srv:22` replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp:1200` replace `mode_sent` with `success` 
   - at `mavros/mavros/src/plugin/sys_status.cpp:1220` replace `mode_sent` with `success` 
   - at `mavros/mavros_msgs/msg/OverrideRCIn.msg:9` replace `uint16[18]` with `uint16[8]` 
6. rebuild: `catkin build`
   