# Setting Up Turtlebot 4
## Resources
Follow these [steps](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)on the ROS 2 Documentation Page in install ROS2 on your Remote PC, laptop. Then follow these [steps](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html) in the official Turtlebot 4 documentation to install all the Turtlebot 4 programs on your Remote PC, laptop. 

## Quick Tip
add the `source /opt/ros/humble/setup.bash` command to the `.bashrc` file so you do not have to source it everytime you open a new terminal window
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Once everything is install follow the instructions to connect the Turtle bot to the wifi. The WIFI password for the turtlebot access point is `Turtlebot4` and the ssh password is `turtlebot4`. To change the settings on the Create 3 go to the Turtlebot's ip address with the port `8080` and you will see the web interface. 