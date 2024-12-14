# Sensors

## How to use Sensors in URDF and Gazebo
Adding sensors in a URDF for Gazebo is not to difficult if you know the command but it can be very time consuming if you get something wrong. Below is an example of how to setup a generic sensor. You can go [here](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins) for more help.
```xml
<gazebo reference="sensor_link">
	<!-- Set the material for the sensor -->
	<material>Gazebo/Red</material>
	<sensor name="sensor_name" type="sensor_type">
		<!-- Pose of the sensor: x y z r p y -->
		<pose>0 0 0 0 0 0</pose>
		<!-- Visualize the sensor in Gazebo -->
		<visualize>true</visualize>
		<!-- Update rate in Hz -->
		<update_rate>10.0</update_rate>
		<plugin name="sensor_controller" filename="libgazebo_ros_sensor.so">
			<!-- Reference link for the sensor -->
			<frame_name>sensor_link</frame_name>
		</plugin>
	</sensor>
</gazebo>
```

## Types of Sensors and configurations
There are many different sensors that can be used in gazebo and in a URDF file and a few of them are listed below. To find more sensor drivers go to the [ros_gazebo github page](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins)
### Cameras
To add a camera to a URDF robot and make it work in gazebo you will have to take advantage of the gazebo features in xacro files. below is a sample of how to setup a camera in gazebo.
```xml title="camera.xacro" linenums="1" 
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

</robot>-
```
#### Opencv Camera Fix
In opencv, z is pointing into the image (the blue axis), x is right (the red axis), and y is down (green axis), while in the gazebo camera x is pointing into the image and z is up, y is right which is similar to the robot convention of x being forward and z up. To fix this we have to create a new link and rotate it so that everything lines up properly. To do this add the new link and joint as show below and then change the frame name to the new `camera_link_optical`.

```xml linenums="1"  hl_lines="1-2 4-11 20"
<link name="camera_link_optical">
</link>
    
<joint name="camera_optical_joint" type="fixed">
	<!-- these values have to be these values otherwise the gazebo camera
		image won't be aligned properly with the frame it is supposedly
		originating from -->
	<origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
	<parent link="camera_link"/>
	<child link="camera_link_optical"/>
</joint>

<gazebo reference="camera_link">
	<material>Gazebo/Red</material>
	<sensor name="camera_sensor" type="camera">
		<pose>0 0 0 0 0 0</pose>
		<visualize>true</visualize>
		<update_rate>10.0</update_rate>
		<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
			<frame_name>camera_link_optical</frame_name>
		</plugin>
	</sensor>
</gazebo>
```
