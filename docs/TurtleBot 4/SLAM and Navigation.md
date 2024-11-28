# SLAM
### Troubleshooting
#### Date and Time off
one of the major problems with the Turtlebot 4 and communication is the time. Make sure you use the `date` command and check what time it is. If the time is off use `rasp-config` to set the correct timezone. The standard Turtlebot 4 image does not have rasp-config install so you will probably have to install it with `sudo apt install rasp-config`.

#### The map keeps getting all messed up
try driving slower and not using the R1 shoulder button that puts the turtlebot in high speed.

#### The controller keeps disconnecting
Try pairing the controller to the computer so that you do not have to stay by the robot as it drives and maps. To pair the controller to the computer and drive it install the joy node.
```bash
sudo apt install ros-humble-turtlebot4-bringup
```
Then pair the controller to the computer like normal. Once the controller is paired start the node by running this command.
```bash
ros2 launch turtlebot4_bringup joy_teleop.launch.py namespace:=/
```
Once this command is running then you should be able to drive the robot like normal


### Launching the SLAM command
Here is the `slam.yaml` config file. You can change the resolution by changing the resolution parameter, by default it is 0.05 but i have had great success when running synchronous SLAM with values as low as 0.01.
```yaml title="slam.yaml"
slam_toolbox:
  ros__parameters:

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: scan
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02 #if 0 never publishes odometry
    map_update_interval: 0.5
    resolution: 0.01 # This is were you can ajust the resolution of the lidar scan
    max_laser_range: 12.0 #for rastering images
    minimum_time_interval: 0.25
    transform_timeout: 0.2
    tf_buffer_duration: 30.
    stack_size_to_use: 40000000 #// program needs a larger stack size to serialize large maps
    enable_interactive_mode: true

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.0
    minimum_travel_heading: 0.0
    scan_buffer_size: 20
    scan_buffer_maximum_scan_distance: 12.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

To start the SLAM node using this command on the remote PC. If this command is run on the Turtlebot 4 you will have a smother experience and will not have as much trouble with connecting but this will limit your resolution because the raspberry pi 4 will have to do all the calculations.
```
ros2 launch turtlebot4_navigation slam.launch.py params:=/full/path/to/slam.yaml
```
To visualize the map run Rviz on the remote PC.
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

Once you are done creating the map make sure you save it.
```bash
ros2 run nav2_map_server map_saver_cli -f "map_name"
```


#### Navigation
To launch the navigation program on the Turtlebot 4 run the following commands. One per terminal window making sure to execute them on the Remote PC.
```bash
# Terminal Window 1
ros2 launch turtlebot4_navigation localization.launch.py map:/path/to/map.yaml

# Terminal Window 2
ros2 launch turtlebot4_navigation nav2.launch.py

# Terminal Window 3
ros2 launch turtlebot4_viz view_robot.launch.py
```

Use the `2D Pose Estimate` to select the spot on the map were the robot is at when the Navigation is launched. Then use the `Nav2 Goal` button to select where the robot should navigate to.