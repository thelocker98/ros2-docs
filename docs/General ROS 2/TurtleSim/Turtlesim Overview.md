# Turtlesim Overview
## What is Turtlesim
Turtlesim is a simple python node that helps you learn ROS 2 with simple examples and tutorials.

## Installation
To install the ROS 2 Turtlesim you will need run `sudo apt install ros-humble-turtlesim`.
```bash
sudo apt install ros-humble-turtlesim
```


## Basic Features of Turtlesim
The Turtlesim package comes with two simple nodes a `turtlesim_node` and a `turtle_teleop_key` node. The `turtlesim_node` launches a window that has a little 2D turtle in it. The `turtle_teleop_key` node allows you to use the arrow keys to move and steer the little turtle around its 2D world. These can be launched using their respective commands.

```bash
ros2 turtlesim turtlesim_node
ros2 turtlesim turtle_teleop_key
```
