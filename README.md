# Differential Drive Robot Simulation

## About

This package provides a simple differential drive robot model designed for use in Gazebo Harmonic simulation with ROS 2 Jazzy Jalisco. 

## Requirements

To run this package, you'll need the following:

- [Linux Ubuntu 24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) 


#### Install Required ROS 2 Packages

Make sure to install the following ROS 2 Jazzy Jalisco packages:

```bash
sudo apt install -y                         \
    ros-jazzy-ros-gz                        \
    ros-jazzy-ros-gz-bridge                 \
    ros-jazzy-joint-state-publisher         \
    ros-jazzy-xacro                         \
    ros-jazzy-teleop-twist-keyboard         \
    ros-jazzy-teleop-twist-joy 
```

## Usage

### Clone the Repository

Clone this repository into your ``workspace/src`` folder. If you don't have a workspace set up, you can learn more about creating one in the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).


```bash
cd <path_to_your_workspace>/src
git clone git@github.com:lucasmazz/gazebo_differential_drive_robot.git
cd ..
```

### Build the Package

Source the ROS 2 environment and build the package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

### Launch the Robot

After building the package, launch the ```robot.launch.py``` file from the ```gazebo_differential_drive_robot``` package:

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

