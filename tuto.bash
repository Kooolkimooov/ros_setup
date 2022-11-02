
echo "\n\n Installing ROS \n\n"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

sudo apt install ros-melodic-desktop-full

echo "\n\n Hooking up sourcing \n\n"

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc

source ~/.bashrc
source ~/.zsh

echo "\n\n Installing Dependencies for building packages \n\n"

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo rosdep init && rosdep update

echo "\n\n Setting up Catkin workspace \n\n"

mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/

echo "source devel/setup.bash" >> ~/.bashrc
echo "source devel/setup.zsh" >> ~/.zshrc

source ~/.bashrc
source ~/.zsh

echo "\n\n Installing some basic packages for bluerovs \n\n"

sudo apt install ros-melodic-mavros
sudo apt install ros-melodic-joy