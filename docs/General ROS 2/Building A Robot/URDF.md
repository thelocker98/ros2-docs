# URDF
## What is a URDF
A **URDF (Unified Robot Description Format)** in ROS 2 is an XML file format used to describe a robot's structure. It defines the robot's physical properties, such as its links (rigid parts), joints (connections), dimensions, and more. URDFs are crucial for simulation, visualization, and robot control as they provide the framework for tools like RViz and Gazebo to understand the robot's design.

here is a sample:
```xml title="my_robot.udrf" linenums="1"
<?xml version="1.0"?>
<robot name="my_robot">
    <material name="blue">
       <color rgba="0 0 0.5 1"/>
    </material>
    <material name="gray">
       <color rgba="0.5 0.5 0.5 1.0"/>
    </material>

    <link name="base_footprint">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </link>

    <link name="base_link">
        <visual>
                <geometry>
                    <box size="0.6 0.4 0.2"/>
                </geometry>
                <origin xyz="0 0 0.1" rpy="0 0 0"/>
                <material name="blue"/>
        </visual>
    </link>

    <link name="left_wheel_link">
        <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
                <origin xyz="0 0 0" rpy="1.57 0 0"/>
                <material name="gray"/>
        </visual>
    </link>


    <link name="right_wheel_link">
        <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
                <origin xyz="0 0 0" rpy="1.57 0 0"/>
                <material name="gray"/>
        </visual>
    </link>

    <link name="caster_wheel_link">
        <visual>
                <geometry>
                    <sphere radius="0.05" />
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <material name="gray"/>
        </visual>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 0.1" rpy="0 0 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="-0.15 0.225 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="-0.15 -0.225 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="0.2 0 -0.05" rpy="0 0 0" />
    </joint>
</robot>
```
### Materials
Materials allow color and other information to be added to a `<visual>`
```xml linenums="1"
<material name="green">
	<color rgba='0 1 0 0'>
</material>
```


### Links
**Links** are fundamental building blocks in a URDF file that represent the robot's physical parts. They allow you to create the individual parts of the robot and they are made up of multiple parts.
#### Visual
1. **`origin`**:
	- Specifies the position and orientation of the visual geometry relative to the link's frame.
	- Example: `<origin xyz="0 0 0.5" rpy="0 0 0" />`.
2. **`geometry`**:
	- Describes the shape of the link. Shapes can be:
		- **Box**: `<box size="x y z" />` specifies dimensions.
		- **Cylinder**: `<cylinder length="h" radius="r" />`.
		- **Sphere**: `<sphere radius="r" />`.
		- **Mesh**: `<mesh filename="path/to/file" scale="x y z" />` for complex shapes.
3. **`material`**:
	- Specifies the color or texture of the link.
	- **Color**: `<color rgba="r g b a" />` uses RGBA values (alpha for transparency).
	- **Texture**: `<texture filename="path/to/texture" />` for applying an image.

```xml linenums="1"
<link name="base_link">
	<visual>
		<geometry>
			<box size="0.6 0.4 0.2" />
		</geometry>
		<origin xyz="0 0 0.1" rpy="0 0 0"/>
		<material name="green" />
	</visual>
</link>
```

#### Inertial
The inertia element is used to calculate the movement of the robot in gazebo. The inertia element is a requirement for the gazebo simulation. To get the 3D tensor for the inertia values use this [Wikipedia link](https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors). I is usually best to put these inertia calculations in xacro function to allow them to be use all over the robot like shown.

```xml linenums="1"
<xacro:macro name="box_inertia" params="m l w h xyz rpy">
	<inertial>
		<mass value="${m}"/>
		<origin xyz="${xyz}" rpy="${rpy}"/>
		<inertia ixx="${(m/12) * (h*h + l*l)}" ixy="0" ixz="0"
				 iyy="${(m/12) * (w*w + l*l)}" iyz="0"
				 izz="${(m/12) * (w*w + h*h)}" />
	</inertial>
</xacro:macro>

<link name="base_link">
	<visual>
		<geometry>
			<box size="${base_length} ${base_width} ${base_height}"/>
		</geometry>
		<origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
		<material name="blue"/>
	</visual>
	<collision>
		<geometry>
			<box size="${base_length} ${base_width} ${base_height}"/>
		</geometry>
		<origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
	</collision>
	<xacro:box_inertia m="5.0" l="${base_length}" w="${base_width}" h="${base_height}"
						xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
</link>
```

##### Bug
Sometimes the inertial values have to be increased to fix drifting in gazebo. To do this usual you just multiply all the inertial input by `2` or `3`.
```xml linenums="1"
<xacro:box_inertia m="5.0" l="${base_length * 2}" w="${base_width * 2}" h="${base_height * 2}"
					xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
```
#### Collisions
The collision element is added inside the link element to give gazebo the information on the collision shape of the robot. This is usually a simplify shape to make the simulation easy on the computer.
```xml linenums="1"
<collision>
	<geometry>
		<cylinder radius="4" length="4"/>
	</geometry>
	<origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
</collision>
```


### Joints
Joints are what old different objects together. There are many different types of joints as shown below. There are also different parts to a joint. The `<parent>` link tells the join what it is attached to while the `<child>` link tells it what its child is. The `<origin>` section tells the joint where it is located with the `xyz`being the coordinates and the `rpy` being role, pitch, and yaw. The `<axis>` section indicates a 1 for allowed moment and a 0 for no movement. Finally the `<limit>` section gives a lower and upper bound.
#### revolute
A hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits that are in radians.
```xml
<joint name="joint_name" type="revolute">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0, 0, 1" />
    <limit lower="0" upper="0.2" velocity="100" effort="100" />
</joint>
```
#### continuous
A continuous hinge joint that rotates around the axis and has no upper and lower limits.
```xml
<joint name="joint_name" type="continuous">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <axis xyz="0, 0, 0" />
    <limit velocity="100" effort="100" />
</joint>
```
#### prismatic
A sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits in meters.
```xml
<joint name="joint_name" type="prismatic">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1, 0, 0" />
    <limit lower="0" upper="0.2" velocity="100" effort="100" />
</joint>
```
#### fixed
This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the `<axis>`, `<calibration>`, `<dynamics>`, `<limits>` or `<safety_controller>`.
```xml
<joint name="joint_name" type="fixed">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
#### floating
This joint allows motion for all 6 degrees of freedom.
```xml
<joint name="joint_name" type="floating">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
#### planar
This joint allows motion in a plane perpendicular to the axis.
```xml
<joint name="joint_name" type="planar">
	<parent link="parent_name"/>
    <child link="child_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0, 0, 0" />
    <limit lower="0" upper="0.2" velocity="100" effort="100" />
</joint>
```

### Meshes
This allows you to import meshes into your URDF from CAD programs. This allows you to add complex parts to your robot.
```xml
<link name="base_link">
	<visual>
		<geometry>
			<mesh filename="package://package/meshes/folder_name/filename.stl" scale="0.001 0.001 0.001"/>
		</geometry>
		<origin xyz="0 0 0.1" rpy="0 0 0"/>
		<material name="green" />
	</visual>
</link>
```
## Links For Help
Here are some links for help.
- [ROS.org](https://wiki.ros.org/urdf/XML)