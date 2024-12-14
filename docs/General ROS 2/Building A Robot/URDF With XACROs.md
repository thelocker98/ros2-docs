# URDF With XACRO
## What is XACRO
**URDF** (Unified Robot Description Format) files are often used to describe the physical configuration of a robot. **Xacro** (XML Macros) is an extension of URDF that allows for modularity and reuse of code through macros and parameters. Xacro simplifies complex URDF files by reducing repetition and enabling parameterization.

When using Xacros in URDF is it important to remember to add it to the XML file at the beginning.
```xml linenums="1"
<?xml version="1.0"?>
<robot name="my_robot1" xmlns:xacro="http://www.ros.org/wiki/xacro">
	...
</robot>
```

## XACRO Features
Below is a list of the common features of XACRO and how they can be used.
### Variables, Parameters, and Math
In XACRO use `xacro:property` to define variables that can be reused or parameterized in the Xacro file. Variables make the URDF more maintainable and flexible. You can math on these store values to help keep the robot dynamic.

```xml linenums="1"
<?xml version="1.0"?>
<robot name="my_robot1" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<xacro:property name="base_length" value="0.6"/>
	<xacro:property name="base_width" value="0.4"/>
	<xacro:property name="base_height" value="0.2"/>
	
	<link name="base_link">
		<visual>
			<geometry>
				<box size="${base_length} ${base_width} ${base_height}"/>
			</geometry>
			<origin xyz="0 0 0.1" rpy="${pi / 2} 0 0"/>
		</visual>
	</link>
</robot>
```

### Macros
Xacro allows you to define reusable blocks of XML code using macros. Macros are defined with `<xacro:macro>` and given a name, then they can be used to repeat a section of XML code.

```xml linenums="1"
<xacro:macro name="example_macro" params="prefix">
	<link name="${prefix}_wheel_link">
		<visual>
			<geometry>
				<cylinder radius="${wheel_radius}" length="${wheel_length}"/>
			</geometry>
			<origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
			<material name="gray"/>
		</visual>
	</link>
</xacro:macro>

<xacro:example_macro prefix="right" />
<xacro:example_macro prefix="left" />

```
### Conditionals
Xacro allows the use of `if` and `unless` for conditional logic. This is useful for enabling and disabling features or customizing configurations based on parameters.

```xml linenums="1"
<xacro:property name="use_lidar" value="true" />

<xacro:if value="${use_lidar}">
	<link name="lidar_link">
	<!-- Lidar geometry -->
	</link>
</xacro:if>
```

### Loops
XACRO also supports looping with `xacro:for`, enabling dynamic generation of repeated elements like links or joints.
```xml linenums="1"
<xacro:property name="num_wheels" value="4" />

<xacro:for each="i" end="${num_wheels - 1}">
	<link name="wheel_${i}">
		<visual>
			<geometry>
				<cylinder radius="0.1" length="0.05" />
			</geometry>
		</visual>
	</link>
</xacro:for>
```

### Include and Modularity
Xacro files can include other Xacro or URDF files using `<xacro:include>`. This enables modularity, where different parts of a robot description can be stored in separate files.

```xml linenums="1"
<xacro:include filename="common_parts.xacro" />
<xacro:call macro="common_part" />
```

## Example XACRO File
Here is a example of a URDF file for a small robot that uses XACRO.

This is the main file:
```xml title="my_robot.urdf.xacro" linenums="1"
<?xml version="1.0"?>
<robot name="my_robot1" xmlns:xacro="http://www.ros.org/wiki/xacro">
	<xacro:include filename="common_properties.xacro" />
	<xacro:include filename="mobile_base.xacro" />
	<xacro:include filename="mobile_base_gazebo.xacro" />
	<xacro:include filename="camera.xacro" />
</robot>
```

This is the file the contains the common properties:
```xml title="common_properties.xacro" linenums="1"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

   <material name="blue">
      <color rgba="0 0 0.5 1"/>
   </material>

   <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
   </material>

   <xacro:macro name="box_inertia" params="m l w h xyz rpy">
      <inertial>
         <mass value="${m}"/>
         <origin xyz="${xyz}" rpy="${rpy}"/>
         <inertia ixx="${(m/12) * (h*h + l*l)}" ixy="0" ixz="0"
                  iyy="${(m/12) * (w*w + l*l)}" iyz="0"
                  izz="${(m/12) * (w*w + h*h)}" />
      </inertial>
   </xacro:macro>

   <xacro:macro name="cylinder_inertia" params="m r h xyz rpy">
      <inertial>
         <mass value="${m}"/>
         <origin xyz="${xyz}" rpy="${rpy}"/>
         <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy="0" ixz="0"
                  iyy="${(m/12) * (3*r*r + h*h)}" iyz="0"
                  izz="${(2/m) * (r*r)}" />
      </inertial>
    </xacro:macro>

   <xacro:macro name="sphere_inertia" params="m r xyz rpy">
      <inertial>
         <mass value="${m}"/>
         <origin xyz="${xyz}" rpy="${rpy}"/>
         <inertia ixx="${(2/5) * m * r * r}" ixy="0" ixz="0"
                  iyy="${(2/5) * m * r * r}" iyz="0"
                  izz="${(2/5) * m * r * r}" />
      </inertial>
    </xacro:macro>

</robot>
```

This is the file the contains the mobile base:
```xml title="mobile_base.xacro" linenums="1"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6"/>
    <xacro:property name="base_width" value="0.4"/>
    <xacro:property name="base_height" value="0.2"/>
    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_length" value="0.05"/>


    <xacro:macro name="wheel_macro" params="prefix">
        <link name="${prefix}_wheel_link">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
                <material name="gray"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
            </collision>
            <xacro:cylinder_inertia m="1.0" r="${2*wheel_radius}" h="${2*wheel_length}"
                                    xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
        </link>
    </xacro:macro>


    <link name="base_footprint">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </link>

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
        <xacro:box_inertia m="5.0" l="${2*base_length}" w="${2*base_width}" h="${2*base_height}"
                           xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
    </link>

    <xacro:wheel_macro prefix="right" />
    <xacro:wheel_macro prefix="left" />

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius / 2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <material name="gray"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="${wheel_radius / 2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>
        <xacro:sphere_inertia m="0.5" r="${2*wheel_radius / 2.0}"
                              xyz="0 0 0" rpy="0 0 0"/>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="${base_length / -4.0} ${base_width/2 + wheel_length/2} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="${base_length / -4.0} -${base_width/2 + wheel_length/2} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="${base_length / 3.0} 0 -${wheel_radius / 2}" rpy="0 0 0" />
    </joint>
</robot>
```


This is the file the contains the gazebo configs:
```xml title="mobile_base.xacro" linenums="1"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
   <gazebo reference="base_link">
      <material>Gazebo/Blue</material>
   </gazebo>

   <gazebo reference="right_wheel_link">
      <material>Gazebo/Gray</material>
   </gazebo>

   <gazebo reference="left_wheel_link">
      <material>Gazebo/Gray</material>
   </gazebo>

   <gazebo reference="caster_wheel_link">
      <material>Gazebo/Gray</material>
      <mu1 value="0.1" />
      <mu2 value="0.1" />
   </gazebo>

   <gazebo>
      <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
         <!-- Update rate in Hz -->
         <update_rate>50</update_rate>
         <!-- Wheels -->
         <left_joint>base_left_wheel_joint</left_joint>
         <right_joint>base_right_wheel_joint</right_joint>
         <!-- Size -->
         <wheel_separation>${wheel_length + base_width}</wheel_separation>
         <wheel_diameter>${wheel_radius}</wheel_diameter>
         
         <!-- Ros Topic -->
         <command_topic>cmd_vel</command_topic>
         <!-- output -->
         <publish_odom>true</publish_odom>
         <publish_odom_tf>true</publish_odom_tf>
         <publish_wheel_tf>true</publish_wheel_tf>
         <odometry_topic>odom</odometry_topic>
         <odometry_frame>odom</odometry_frame>
         <robot_base_frame>base_footprint</robot_base_frame>
      </plugin>
   </gazebo>
</robot>
```

This is the file the contains the camera configs:
```xml title="mobile_base.xacro" linenums="1"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="camera_length" value="0.01"/>
    <xacro:property name="camera_width" value="0.1"/>
    <xacro:property name="camera_height" value="0.05"/>


    <link name="camera_link">
       <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <material name="gray" />
        </visual>
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>
        <xacro:box_inertia m="0.1" l="${camera_length}" w="${camera_width}" h="${camera_height}" xyz="0 0 0" rpy="0 0 0"/>
    </link>

    <joint name="base_camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="${(base_length + camera_length)/ 2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>

    <gazebo reference="camera_link">
        <material>Gazebo/Red</material>
        <sensor name="camera_sensor" type="camera">
            <!-- x y z r p y-->
            <pose>0 0 0 0 0 0</pose>
            <!-- whether you can see projection in gazebo or not-->
            <visualize>true</visualize>
            <!-- update rate in hz-->
            <update_rate>10.0</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <!-- camera link -->
                <frame_name>camera_link</frame_name>
            </plugin>
         </sensor>
    </gazebo>

</robot>
```