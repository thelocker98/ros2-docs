# Husky 200 Simulator
## Launching Simulator
To launch Gazbo with the Husky 200 robot in it run the following command.
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath
```

Launch MoveIt! by passing the same robot setup directory and setting the simulation flag.
```
ros2 launch clearpath_manipulators moveit.launch.py setup_path:=$HOME/clearpath use_sim_time:=true
```

Launch RViz by passing the robot's namespace and enabling the simulation flag.
```
ros2 launch clearpath_viz view_moveit.launch.py namespace:=a200_0000 use_sim_time:=True
```



