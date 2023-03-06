
This installation guide is aligned with the one already provided by the `README.md` within the root folder of this project.

However, this setup works also on
- Ubuntu 20.04.5 LTS
- ROS Noetic

## Installation


### Step 1: ROS with Gazebo (tested with noetic)

```bash
sudo apt install curl python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```bash
sudo apt install ros-noetic-desktop-full
```

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```bash
sudo apt install python3-rosdep
rosdep update
```

### Step 2: ROS bridge (http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)
```bash
sudo apt-get install ros-noetic-rosbridge-suite
source ~/.bashrc
```

To launch the ROS bridge later: `roslaunch rosbridge_server rosbridge_websocket.launch`

### Step 3: The TurtleBot3 packages (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

```
// The first three are probably not needed
// wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
// chmod 755 ./install_ros_noetic.sh 
// bash ./install_ros_noetic.sh
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3

```

### Step 4: The TurtleBot3 simulation package (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
```bash
cd ~/catkin_ws/src/
git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8d
```

You may need to change the Python paths or remove them completely. This depends on your system's setup, whether you have multiple Python instances running.

## Starting Procedure

- `source ~/catkin_ws/devel/setup.sh`

- See original `README.md` in the root folder of this project

## Errors & Solutions

**When:** Issuing command `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
**Error:** "substitution args not supported:  No module named 'rospkg'"
**Solution:** Uninstall Python anaconda environment