# Gazebo Sensors

## How to use Sensors in URDF and Gazebo
Adding sensors in a URDF for Gazebo is not too difficult if you know the command, but it can be very time-consuming if you get something wrong. Below is an example of how to set up a generic sensor. You can go [here](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins) for more help with gazebo Classic sensors and [here]() for help with Gazebo Fortress sensors.

=== "Gazebo Classic"

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
=== "Gazebo Classic"

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
		<plugin name="sensor::controller" filename="libgazebo::ros::sensor">
			<!-- Reference link for the sensor -->
			<frame_name>sensor_link</frame_name>
		</plugin>
	</sensor>
</gazebo>
```
### Cameras
To add a camera to a URDF robot and make it work in gazebo you will have to take advantage of the gazebo features in XACRO files. Below is a sample of how to set up a camera in gazebo Classic and Gazebo Fortress.


=== "Gazebo Classic"

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
	
	</robot>
	```
=== "Gazebo Fortress"
	
	```xml
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
	
	    <gazebo reference="camera_link">
	        <sensor name="camera" type="camera">
	            <always_on>true</always_on>
	            <visualize>true</visualize>
	            <update_rate>30</update_rate>
	            <topic>camera/image_raw</topic>
	            <gz_frame_id>camera_link</gz_frame_id>
	            <camera name="intel_realsense_r200">
	                <camera_info_topic>camera/camera_info</camera_info_topic>
	                <horizontal_fov>1.02974</horizontal_fov>
	                <image>
	                    <width>1920</width>
	                    <height>1080</height>
	                    <format>R8G8B8</format>
	                </image>
	                <clip>
	                    <near>0.02</near>
	                    <far>300</far>
	                </clip>
	                <noise>
	                    <type>gaussian</type>
	                    <mean>0.0</mean>
	                    <stddev>0.007</stddev>
	                </noise>
	            </camera>
	        </sensor>
	    </gazebo>
	
	
	    <joint name="base_camera_joint" type="fixed">
	        <parent link="base_link"/>
	        <child link="camera_link"/>
	        <origin xyz="${(base_length + camera_length)/ 2} 0 ${base_height/2}" rpy="0 0 0"/>
	    </joint>
	</robot>
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


#### IMU
Here is the Gazebo Fortress code for an IMU:

```xml
<link name="imu_link">

</link>

<gazebo reference="imu_link">
  <sensor name="tb3_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <topic>imu</topic>
    <imu>
      ... <!-- all the content of <imu> -->
    </imu>
  </sensor>
</gazebo>
```

#### Lidar
Here is the Gazebo Fortress code for an Lidar:

```xml
<link name="imu_link">

</link>

<gazebo reference="lidar_link">
	<sensor name="hls_lfcd_lds" type="gpu_lidar">
		<always_on>true</always_on>
		<visualize>true</visualize>
		<pose>-0.064 0 0.121 0 0 0</pose>
		<update_rate>5</update_rate>
		<topic>scan</topic>
		<gz_frame_id>base_scan</gz_frame_id>
		<lidar>
		... <!-- same content as <ray> in the original -->
		</lidar>
	</sensor>
</gazebo>
```
