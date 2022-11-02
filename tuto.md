# ROS installation and setup for Bluerovs

## ROS install

Following [this](http://wiki.ros.org/Installation/Ubuntu) tutorial.

1. run `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. if do not have it already, install curl: `sudo apt install curl`
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

Following [this](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial.

1. create a working directory, for example: `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/`
2. build the catkin worspace: `catkin_make`
3. add either:
   - `source devel/setup.bash` to `~/.bashrc`
   - `source devel/setup.zsh` to `~/.zshrc`

## Other setup

- `sudo apt install ros-melodic-mavros`
- `sudo apt install ros-melodic-joy`