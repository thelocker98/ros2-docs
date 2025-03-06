# General Setup for ROS 1
Setting up Husky A200 with kinetic: https://youtu.be/wA8UTF0mKBY


# ROS 2 Tips and Tricks
## Creating `setup.bash` for `robot.yaml`
Off-board computer setup: https://docs.clearpathrobotics.com/docs/ros/networking/ros2-networking#off

generate bash from `robot.yaml`
```bash
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
```
Then source the file
```
source ~/clearpath/setup.bash
```

## Starting Clearpath ROS Discovery Server
First navigate to your husky configuration directory on your laptop and create server bash file:
```bash
ros2 run clearpath_generator_common generate_discovery_server -s ~/clearpath
```

Then launch the server:
```bash
bash -e ~/clearpath/discovery-server-start 
```

## Viewing Robot
If you want to view the current husky configuration and make changes while watching what happens without reloading everything then run this command:
```bash
ros2 launch clearpath_viz view_robot.launch.py namespace:=a200_0000
```

If you want to see nav2 and the nav2 map for the robot run this command:
```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
```

## SLAM
To start SLAM and Nav2 set up the locker98 workspace that has all the launch files for SLAM and Nav2:
```bash
ros2 launch locker98_tools_bringup husky.slam.launch.py setup_path:=$HOME/husky1/
```

Viewing the SLAM Map:
```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
```

Saving the SLAM map:
```bash
ros2 run nav2_map_server map_save_cli -f path/to/map -t /a200_0651/map
```

## Nav2
Nav2 allows you to navigate through a pre created map and go to different locations. To launch this first start the localization:
```bash
ros2 launch locker98_tools_bringup husky.localization.launch.py setup_path:=$HOME/husky1/ map:=$HOME/husky1/rhodes1.yaml
```

Then launch Nav2
```bash
ros2 launch locker98_tools_bringup husky.nav2.launch.py setup_path:=$HOME/husky1/
```

Finally, you can run Rviz to see the nav2 map and send nav2 goals
```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
```

## Discovery
If you are having trouble with the Husky being discovered by the laptop make sure all the hostname's and ip's are set and in /etc/hosts. Use this [page](https://docs.clearpathrobotics.com/docs/ros/networking/ros2_discovery_config#offboard-pc) for setting up discovery server correctly.

## Updating robot.yaml
Every time you update the robot.yaml or other settings on the Husky A200 you have to restart robot or run `sudo systemctl restart clearpath-robot.service` to restart the Clearpath daemon and create the new configuration files. These files are usually found in `/etc/clearpath` and there is some documentation on this [here](https://docs.clearpathrobotics.com/docs/ros/installation/robot#start-service)
