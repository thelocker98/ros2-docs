# Gazebo Plugins
The gazebo element allows you to specifies information for gazebo. 
## Plugins
The plugin feature of URDF for gazebo lets you add ROS topics to URDF files so that you can simulate different inputs and outputs of values. This is different in Gazebo Classic and Gazebo Fortress as shown below:

### Diff Drive Controller
Here is an example of a plugin for diff drives in Gazebo:

=== "Gazebo Classic"
	```xml
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
	```

=== "Gazebo Fortress"

	```xml
	<gazebo>
	  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
		 <!-- wheels -->
		 <left_joint>base_left_wheel_joint</left_joint>
		 <right_joint>base_right_wheel_joint</right_joint>
	
		 <!-- kinematics -->
		 <wheel_separation>0.287</wheel_separation>
		 <wheel_radius>0.033</wheel_radius>
	
		 <!-- limits -->
		 <max_linear_acceleration>1.0</max_linear_acceleration>
	
		 <topic>cmd_vel</topic>
	
	
		 <odom_topic>odom</odom_topic>
		 <frame_id>odom</frame_id>
		 <tf_topic>/tf</tf_topic>
		 <odom_publisher_frequency>30</odom_publisher_frequency>
		 <child_frame_id>base_footprint</child_frame_id>
	  </plugin>
	</gazebo>
	```

### Joint State Publisher
Here is an example of a plugin for Joint State Publishing in Gazebo:

=== "Gazebo Classic"
	```xml
	<gazebo>
		<plugin name="joint_state_publisher_controller"
			filename="libgazebo_ros_joint_state_publisher.so">
			<!-- Update rate in Hertz -->
			<update_rate>10</update_rate>
			<!-- Name of joints in the model whose states will be published. -->
			<joint_name>arm_base_forearm_joint</joint_name>
			<joint_name>forearm_hand_joint</joint_name>
		</plugin>
	</gazebo>
	```

=== "Gazebo Fortress"
	
	```xml
	<gazebo>
	  <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
		 <topic>joint_states</topic>
		 <update_rate>10</update_rate>
	
		 <!-- Name of joints in the model whose states will be published. -->
		 <joint_name>arm_base_forearm_joint</joint_name>
		 <joint_name>forearm_hand_joint</joint_name>
		 <joint_name>base_left_wheel_joint</joint_name>
		 <joint_name>base_right_wheel_joint</joint_name>
	  </plugin> 
	</gazebo>
	```

### Joint Trajectory Controller 
Here is an example of a plugin for the Joint Trajectory Controller in Gazebo:

=== "Gazebo Classic"

	```xml
	<gazebo>
		<plugin name="joint_pose_trajectory_controller"
			filename="libgazebo_ros_joint_pose_trajectory.so">
			<!-- Update rate in Hz -->
			<update_rate>2</update_rate>
		</plugin>
	</gazebo>
	```

=== "Gazebo Fortress"
	
	```xml
	<!-- Unknow Right Now -->
	```

## Gazebo Fortress World Plugins
For gazebo Fortress there is not only plugins for URDF files but also for the `world.sdf` files. Here is an example of a `world.sdf` file with the basic plugins.

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="car_world">
        <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
        </plugin>
        <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>

        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <scene>
            <ambient>1 1 1 1</ambient>
            <background>0.3 0.7 0.9 1</background>
            <shadows>0</shadows>
            <grid>1</grid>
        </scene>

        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>1 1</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode/>
                        </friction>
                        <bounce/>
                        <contact/>
                    </surface>
                </collision>
            </link>
            <pose>0 0 0 0 0 0</pose>
        </model>
    </world>
</sdf>
```
