# Gazebo
## Installing Gazebo
### Installing Gazebo Classic
To install gazebo classic:
```bash
sudo apt install ros-humble-gazebo-*
```

Sometimes gazebo has the wrong resource path and the correct one has to be added to the `~/.bashrc` file.
```bash
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
```

### Installing Gazebo Latest
To install the latest version of gazebo go to the [official gazebo installation page](https://gazebosim.org/docs/latest/ros_installation/) and follow the instruction for the correct version of ROS.


## Launching Gazebo Classic
To launch gazebo classic simply run `gazebo`:
```bash
gazebo
```

If you want to launch gazebo with ROS2 you will have to use its launch file:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

It can be trick to use a the gazebo launch file in another launch file but it can be done with the following code:
```xml
<include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py" />
```

## Using a robot in Gazebo Classic
To get robot to appear in a Gazebo world first launch gazebo like show previously. Then use the following commands to advertise the robot description and spawn it in gazebo.

Terminal:
```bash
# start gazebo
ros2 launch gazebo_ros gazebo.launch.py
# advertise the robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /path/to/my_robot1.urdf.xacro)"
# spawn robot
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot1
```
launch file:
```xml
<launch>
	<let name="urdf_path" value="$(find-pkg-share my_robot1_description)/urdf/my_robot1.urdf.xacro"/>
	<let name="rviz_config_path" value="$(find-pkg-share my_robot1_bringup)/rviz/urdf_config.rviz"/>
	
	<node pkg="robot_state_publisher" exec="robot_state_publisher">
		<param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
	</node>
	
	<include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
		<arg name="world" value="$(find-pkg-share my_robot1_bringup)/worlds/test_world.world.xml"/>
	</include>
	
	<node pkg="gazebo_ros" exec="spawn_entity.py" args="-topic robot_description -entity my_robot1"/>
	<node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />
</launch>
```

## Building and Launching a World
To build a world in gazebo you can follow this [tutorial](https://youtu.be/9Q2IuoVbqYo) Once the world is build and save it can be launch with the following command.

```bash
gazebo test_world.world.xml
```
or
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=test_world.world.xml
```
or in an XML launch file
```xml
<include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
	<arg name="world" value="$(find-pkg-share my_robot1_bringup)/worlds/test_world.world.xml"/>
</include>
```