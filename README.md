# TurtleBot3_Swarm
Swarm robotics implmeneted on turtlebots in gazebo 

## Overview
This project implements a swarm robotics system using TurtleBot3 robots in a ROS2 Humble environment. The swarm performs the task of searching for an object, (here a green ball) and avoiding obstacles while coordinating to reach the goal in a Gazebo simulation. The primary goal is to explore and demonstrate multi-robot collaboration, detection of objects using a pre-trained machine learning model and dynamic control using Dynamic Window Approach (DWA).

## Features
- Scalable Simulation: Supports spawning multiple TurtleBot3 robots in Gazebo.
- Swarm Synchronization: Robots maintain clear and concise communication for the necessary information to be in coordination and complete task.
- ROS2 Integration: Built with ROS2 Humble for the advanced robotic communication and control.
- Green Ball Detection: Pre-trained model from roboflow imported and used by each bot to detect the green ball.
- Dynamic Window Approach (DWA): The DWA controller is utilized by robots that have not detected the green ball to navigate toward the position of the robot that detected it. This also helps it to avoid obstacles.

## Workflow
1. Four bots are spawned in their given positions in the gazebo house world.
2. All the bots search for the green ball in their surroundings using the pre-trained model.
3. As soon as one of the bots detects the green ball, all of them stop and the position of that bot is given to the other bots over a ros2 topic.
4. All the other bots attempt to reach that goal position using DWA controller.

## Requirements
- ROS2 Humble
- Gazebo11
- Turtlebot3 Gazebo Package

## Installation and Setup
### Step1: Clone the repository
```bash
cd ~/path/to/desired/location
git clone https://github.com/Janhavi-118/TurtleBot3_Swarm.git
```

### Step2: Installing Dependencies
- ROS2 Humble : To install ROS2 Humble, [click here](https://docs.ros.org/en/humble/Installation.html) and follow the instructions.
- Gazebo11 : To install Gazebo11, [click here](https://gazebosim.org/docs/latest/install_gz11_side_by_side/) and follow the instructions.
- Turtlebot3 : 
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3*
```
If you are facing errors in this way, install from source according to the [official documentation](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)

### Step3: Sourcing
- ROS2 Humble: 
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
- Gazebo11 and turtlebot3
```bash
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/opt/ros/humble/local/lib" >> ~/.bashrc
```

## Execution
Open two terminals:
1. First one will be used to run the launch file which spawns the bots.
Run the following commands:
```bash
cd ~/path/to/TurtleBot3_Swarm
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch swarm_locate spawn_turtles.launch.py
```
Waffle model of the turtlebot3 is used since it has the camera module as well.

2. Second one will launch the swarm behaviour script
Run the following commands:
```bash
cd ~/path/to/TurtleBot3_Swarm
source install/setup.bash
ros2 launch swarm_locate swarm_behaviour.launch.py 
```


