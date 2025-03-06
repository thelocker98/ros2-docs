# Gazebo Fortress
## Installing Gazebo Fortress
To install the latest version of gazebo go to the [official gazebo installation page](https://gazebosim.org/docs/fortress/install/) and follow the instruction for the correct version of ROS.

First install some necessary tools:
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

Then install Ignition Fortress:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

## Launching Gazebo Fortress
To launch gazebo classic simply run `ign gazebo`:
```bash
ign gazebo
```

If you want to launch gazebo fortress with ROS2 you will have to use its launch file:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

It can be trick to use gazebo fortress launch file in another launch file, but it can be done with the following code:
```xml
<include file="$(find-pkg-share ros_gz_sim)/launch/gz_sim.launch.py"/>
```

## Using a robot in Gazebo Fortress
To get robot to appear in a Gazebo world first launch gazebo fortress like show previously. Then use the following commands to advertise the robot description and spawn it in gazebo fortress.

Terminal:
```bash
# start gazebo
os2 launch ros_gz_sim gz_sim.launch.py
# advertise the robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /path/to/my_robot1.urdf.xacro)"
# spawn robot
ros2 run ros_gz_sim create -topic robot_description -name my_robot1
```
launch file:
```xml
<launch>
    <let name="urdf_path" value="$(find-pkg-share my_robot1_description)/urdf/my_robot1.urdf.xacro"/>
    <let name="rviz_config_path" value="$(find-pkg-share my_robot1_description)/rviz/urdf_config.rviz"/>

    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
    </node>

    <include file="$(find-pkg-share ros_gz_sim)/launch/gz_sim.launch.py">
        <arg name="gz_args" value="-r -v4 $(find-pkg-share my_robot1_bringup)/worlds/world.sdf"/>
    </include>
    
    <node pkg="ros_gz_sim" exec="create" args="-topic robot_description -name my_robot1 "/>

    <node pkg="ros_gz_bridge" exec="parameter_bridge" args="--ros-args -p config_file:=$(find-pkg-share my_robot1_bringup)/params/gz_ros_bridge.yaml"/>
    <node pkg="ros_gz_image" exec="image_bridge" args="/camera/image_raw"/>

    <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)"/>
</launch>
```

## Building and launching a World
To build a world in gazebo fortress you can use [fuel](https://app.gazebosim.org/dashboard), the new gazebo asset store. Once the world is build and save it can be launched with the following command.

```bash
ign gazebo world.sdf
```
Or
```bash
ros2 launch ros_gz_sim gz_sim.launch.py --gz_args -r -v4 world.sdf
```
or in an XML launch file
```xml
<include file="$(find-pkg-share ros_gz_sim)/launch/gz_sim.launch.py">
	<arg name="gz_args" value="-r -v4 $(find-pkg-share my_robot1_bringup)/worlds/world.sdf"/>
</include>
```

## Gazebo Fortress Tools
Gazebo Fortress comes with lots of tools to help see what is going on and integrate it with ROS 2. Gazebo Fortress comes with ignition which as the tooling.

If you want to see all the Gazebo topics then just run:
```
ign topic -l
```

If you want to see the contents of this topic you can run:
```
ign topic -t /cmd_vel -e
```

And if you want to get the info about it then just run:
```
ign topic -t /cmd_vel -i
```

For help run:
```
ign -h
```


## Gazebo ROS Bridge
Since the Gazebo topics are separate from the ROS topic there has to be a bridge. This can be done in multiple ways. You can use a YAML file to describe the bridge or with use ROS2 CLI commands that does topics individual. The documentation on the ROS2 bridge can be found [here](https://gazebosim.org/docs/fortress/ros2_integration/) and documentation on compatible topic types can be found [here](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-ignition-transport-talker-and-ros-2-listener).

### Individual
Here is how the individual topi

```bash
ros2 run ros_gz_bridge parameter_bridge /keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32
```
The ROS message type is followed by an `@`, `[`, or `]` symbol where:
- `@` is a bidirectional bridge.
- `[` is a bridge from Ignition to ROS.
- `]` is a bridge from ROS to Ignition.
- 
### YAML file
With a launch file you will first create a YAML file to define the bridge:
```yaml
- ros_topic_name: "scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "ignition.msgs.LaserScan"
  direction: IGN_TO_ROS  # BIDIRECTIONAL or ROS_TO_IGN
```

Then just call this file using the ROS 2 Package:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="gz_ros_bridge.yaml"
```