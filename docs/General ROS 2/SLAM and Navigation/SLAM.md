# SLAM
## What is SLAM
**SLAM** stands for **Simultaneous Localization and Mapping**. It is a computational problem in robotics and computer vision where a robot or device:
1. **Localizes itself**: Determines its position and orientation within an environment.
2. **Maps the environment**: Creates a map of its surroundings while navigating through it.

SLAM enables robots to operate autonomously in unknown environments by building a map and tracking their position on that map in real-time.

## Starting SLAM from the Terminal
Here are the terminal commands for launching the SLAM toolkit.

```bash
# Start robot or robot simulator in this case I wil use the turtlebot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Start nav2
ros2 launch nav2_bringup navigation_launch.py
# Start SLAM
ros2 launch slam_toolbox online_async_launch.py
# Launch rviz2 and add the TF visualization and the map visualization
ros2 run rviz2 rviz2
```

## Custom SLAM Launch File
Here is a custom ROS 2 Launch file

=== "Python"

	```python title="slam.launch.py" linenums="1"
	from launch import LaunchDescription
	from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
	from launch_ros.actions import Node
	from ament_index_python.packages import get_package_share_directory
	from launch.launch_description_sources import PythonLaunchDescriptionSource
	from launch.substitutions import LaunchConfiguration
	import os
	
	def generate_launch_description():
	    # Declare arguments
	    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
	
	    # Get package share directories
	    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
	    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
	    locker98_tools_bringup_dir = get_package_share_directory('locker98_tools_bringup')
	
	    # Include the navigation launch file
	    navigation_launch = IncludeLaunchDescription(
	        PythonLaunchDescriptionSource(
	            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
	        ),
	        launch_arguments={'use_sim_time': use_sim_time}.items()
	    )
	
	    # Include the SLAM toolbox launch file
	    slam_launch = IncludeLaunchDescription(
	        PythonLaunchDescriptionSource(
	            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
	        ),
	        launch_arguments={'use_sim_time': use_sim_time}.items()
	    )
	
	    # RViz2 node
	    rviz2_node = Node(
	        package='rviz2',
	        executable='rviz2',
	        output='screen',
	        arguments=[
	            '-d', os.path.join(locker98_tools_bringup_dir, 'rviz', 'slam_config.rviz')
	        ]
	    )
	
	    return LaunchDescription([
	        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
	        navigation_launch,
	        slam_launch,
	        rviz2_node
	    ])
	```

=== "XML"
	
	This is the XML version that is much easier to understand but for some reason it does not work. There is some error with the args the makes it error every time I try to build it.
	
	```xml title="slam.launch.xml" linenums="1"
	<launch>
		<arg name="use_sim_time" default="False" />
		
		<include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
			<arg name="use_sim_time" value="$(arg use_sim_time)"/>
		</include>
		
		<include file="$(find-pkg-share slam_toolbox)/launch/online_async_launch.py">
			<arg name="use_sim_time" value="$(arg use_sim_time)"/>
		</include>
		
		<node pkg="rviz2" executable="rviz2" output="screen" args="-d $(find-pkg-share locker98_tools_bringup)/rviz/nav2_slam.rviz" />
	</launch>
	```
	
