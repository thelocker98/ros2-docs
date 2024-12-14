# ROS 2 Launch Files
**Launch files** are Python-based scripts that manage the startup of multiple nodes and configurations simultaneously. They allow users to define which nodes to launch, pass parameters, remap topics, and set arguments, streamlining the deployment and orchestration of complex robotics systems.


## Creating Launch File Package
When creating a launch file in ROS 2 you have to use the `ros2 pkg create package_name`. Once the package is created delete all the folders in the package folder and create a folder called `launch` inside this folder you can put all the launch files. Finally edit the `CMakeLists.txt` so that it looks like the one below.

```cmake  linenums="1"
cmake_minimum_required(VERSION 3.8)
project(my_robot1_description)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

## Creating Python Launch File
Below is a example of a launch file. 
=== "Python"

	```python title="display.launch.py" linenums="1"
	from launch import LaunchDescription
	from launch_ros.parameter_descriptions import ParameterValue
	from launch.substitutions import Command
	from launch_ros.actions import Node
	from ament_index_python import get_package_share_directory
	import os
	
	
	def generate_launch_description():
		urdf_path = os.path.join(get_package_share_directory('my_robot1_description'), 'urdf', 'my_robot1.urdf')
		robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
		rviz_config_path = os.path.join(get_package_share_directory('my_robot1_description'), 'rviz', 'urdf_config.rviz')
		
		
		robot_state_publisher_node = Node(
			package="robot_state_publisher",
			executable="robot_state_publisher",
			parameters = [{"robot_description": robot_description}]
		)
	
	
		joint_state_publisher_gui_node = Node(
			package="joint_state_publisher_gui",
			executable="joint_state_publisher_gui",
		)
	
	
		rviz2_node = Node(
			package="rviz2",
			executable="rviz2",
			arguments=['-d', rviz_config_path]
		)
		
		return LaunchDescription([
			robot_state_publisher_node,
			joint_state_publisher_gui_node,
			rviz2_node
		])
	```
	
=== "XML"

	```xml title="display.launch.xml" linenums="1"
	<launch>
		<let name="urdf_path" value="$(find-pkg-share my_robot1_description)/urdf/my_robot1.urdf"/>
		<let name="rviz_config_path" value="$(find-pkg-share my_robot1_description)/rviz/urdf_config.rviz"/>
		
		<node pkg="robot_state_publisher" exec="robot_state_publisher">
			<param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
		</node>
		
		<node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" />
		
		<node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />
	
	</launch>
	```

## Adding configs and UFDRs in Launch File Args
When you need to add a file or a config like when creating a rviz node you can take a few quick steps. First create another folder in the same directory you have the launch file. Next navigate to the `CMakeList.txt` file and and add a space followed by the name of the folder you create. In this case I am creating an folder called `rviz` and I will add it to the end of line `12`.

```cmake title="CMakeLists.txt" linenums="1" hl_lines="12"
cmake_minimum_required(VERSION 3.8)
project(my_robot1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch rviz
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

Now once it is added you will need to reference it in you launch file. Doing this is slightly different in python and xml but it is not two hard.

=== "Python"

	```python
	from launch import LaunchDescription
	from launch_ros.actions import Node
	from ament_index_python import get_package_share_directory
	import os
	
	
	def generate_launch_description():
		rviz_config_path = os.path.join(get_package_share_directory('my_robot1_description'), 'rviz', 'urdf_config.rviz')
		
		rviz2_node = Node(
			package="rviz2",
			executable="rviz2",
			arguments=['-d', rviz_config_path]
		)
		return LaunchDescription([rviz2_node])
	```

=== "XML"

	```xml
	<launch>
		<let name="rviz_config_path" value="$(find-pkg-share my_robot1_description)/rviz/urdf_config.rviz"/>
		<node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />
	</launch>
	```
	
## Using Launch File
To run a launch file use the `ros2 launch` command.
```bash
ros2 launch package_name file.launch.py
```
