# Turtlebot 4 ROS 2 Commands
## Undock
This command undocks the Turtlebot 4 from the charging station.
```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```
## Dock
This command docks the Turtlebot 4 from the charging station.
```bash
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
```
