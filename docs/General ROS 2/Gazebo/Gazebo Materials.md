# Gazebo Material
Material textures and colors can be added in UDRF files to customize how the robot behaves in Gazebo. This is handled differently in Gazebo Classic and Gazebo Fortress. Below you can see both ways. The main difference is that Gazebo Classic must have a material to give the object color whereas in Gazebo Fortress the Color is just taken from the UDRF matrial.

=== "Gazebo Classic"

	```xml
	<!-- the reference field gives gazebo the URDF link element  -->
	<gazebo reference="link_name">
		<!-- Material Color -->
		<material>Gazebo/Gray</material>
		<!-- friction values -->
		<mu1 value="0.1" />
		<mu2 value="0.1" />
	</gazebo>
	```
=== "Gazebo Fortress"

	```xml
	<!-- the reference field gives gazebo the URDF link element  -->
	<gazebo reference="link_name">
		<!-- friction values -->
		<mu1 value="0.1" />
		<mu2 value="0.1" />
	</gazebo>
	```
