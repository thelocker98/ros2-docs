launching simulator
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath
```


## Using Manipulator
robot.yaml
```yaml
serial_number: a200-0000
version: 0
system:
  hosts:
    - hostname: cpr-a200-0000
      ip: 192.168.131.1
  ros2:
    namespace: a200_0000
platform:
  attachments:
    - name: front_bumper
      type: a200.bumper
      parent: front_bumper_mount
    - name: rear_bumper
      type: a200.bumper
      parent: rear_bumper_mount
    - name: top_plate
      type: a200.top_plate
manipulators:
  arms:
    - model: kinova_gen3_lite
      parent: top_plate_default_mount
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      ip: 192.168.131.40
      port: 10000
      gripper:
        model: kinova_2f_lite
```

Launch the simulation
```
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



