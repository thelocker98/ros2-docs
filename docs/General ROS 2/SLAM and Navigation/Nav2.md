# Nav2
## What is Nav2
**Nav2**, or the **Navigation 2 Stack**, is a powerful and flexible navigation framework for robots in the **Robot Operating System 2 (ROS 2)**. It provides a modular set of tools and algorithms for autonomous robot navigation in complex environments.

## Starting Nav2 from the Terminal
Here are the terminal commands for launching the Nav2 toolkit.

```bash
# Start robot or robot simulator in this case I wil use the turtlebot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=Desktop/maps/my_map.yaml
# Launch rviz2 and add the TF visualization and the map visualization
ros2 run rviz2 rviz2
```

## Custom Nav2 Launch File
Here is a custom ROS 2 Launch file

=== "Python"
	
	```python title="nav2.launch.py" linenums="1"
	from launch import LaunchDescription
	import os
	from ament_index_python import get_package_share_directory
	from launch_ros.parameter_descriptions import ParameterValue
	from launch.substitutions import LaunchConfiguration
	from launch_ros.actions import Node
	from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
	
	def generate_launch_description():
	
		# Declare the arguments
		use_sim_time = LaunchConfiguration('use_sim_time', default='False')
		map = LaunchConfiguration('map')
	
		# Define RViz config path
		rviz_config_path = os.path.join(
			get_package_share_directory('locker98_tools_bringup'),
			'rviz', 'nav2_config.rviz'
		)
	
		# Include the bringup launch file
		bringup_launch_file = os.path.join(
			get_package_share_directory('nav2_bringup'),
			'launch', 'bringup_launch.py'
		)
	
		# Include the bringup launch file and pass arguments
		include_bringup = IncludeLaunchDescription(
			bringup_launch_file,
			launch_arguments={
				'use_sim_time': use_sim_time,
				'map': map
			}.items()
		)
	
		# RViz node
		rviz2_node = Node(
			package='rviz2',
			executable='rviz2',
			output='screen',
			arguments=['-d', rviz_config_path]
		)
	
		# Return LaunchDescription
		return LaunchDescription([
			DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
			DeclareLaunchArgument('map', default_value='', description='Map input'),
			include_bringup,
			rviz2_node
		])
	```

=== "XML"
	
	This is the XML version that is much easier to understand but for some reason it does not work. There is some error with the args the makes it error every time I try to build it.
	
	```xml title="nav2.launch.xml" linenums="1"
	<?xml version="1.0"?>
	<launch>
	    <!-- Declare Launch Arguments -->
	    <arg name="use_sim_time" default="false" description="Use simulation time" />
	    <arg name="map" default="" description="Map input" />
	
	    <!-- Include bringup launch file -->
	    <include file="$(find-pkg-share nav2_bringup)/launch/bringup_launch.py">
	        <arg name="use_sim_time" value="$(arg use_sim_time)" />
	        <arg name="map" value="$(arg map)" />
	    </include>
	
	    <!-- RViz node -->
	    <node pkg="rviz2" exec="rviz2" output="screen">
	        <param name="config" value="$(find-pkg-share locker98_tools_bringup)/rviz/nav2_config.rviz" />
	    </node>
	</launch>
	```