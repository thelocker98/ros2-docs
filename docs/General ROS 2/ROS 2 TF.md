# ROS 2 TF
## What is ROS 2 TF
**TF (Transform)** is a library used to keep track of coordinate frames and their relationships over time. It allows robots to understand where objects and parts of themselves are in 3D space, enabling tasks like localization, navigation, and manipulation. For example, it handles transformations between the robot base, sensors, and the world.

## TF Tools
If you want to visualize the relation of the different TF's in a file use the following command to generate a PDF with a mermaid graph.
```bash
ros2 run tf2_tools view_frames
```
```mermaid
graph TD;
	base_link --> left_leg;
	left_leg --> left_base;
	left_base --> left_front_wheel;
	left_base --> left_back_wheel;
	
	base_link --> right_leg;
	right_leg --> right_base;
	right_base --> right_front_wheel;
	right_base --> right_back_wheel;

	base_link --> gripper_pole;
	gripper_pole --> left_gripper;
	left_gripper --> left_tip;
	gripper_pole --> right_gripper;
	right_gripper --> right_tip;

	base_link --> head;
	head --> box;
```


## Viewing TFs
To view a urdf file that has TFs in it use the following command.
```bash
ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/humble/share/urdf_tutorial/urdf/08-macroed.urdf.xacro
```
