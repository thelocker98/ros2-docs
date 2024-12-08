# robot.yaml
The `robot.yaml` file has all the configurations for the Husky 200 robot and the other clearpathrobotics robots.
## Visualizing
To see the `robot.yaml` file run the following command. It will show you how the robot looks and where all the sensors are placed. It comes in hand for not only confirming that everything is on the robot in the right place but that their is also no errors in the robot.yaml file.

```bash
ros2 launch clearpath_viz view_model.launch.py setup_path:=/home/brickman/clearpath/
```


## Sample file
Below is a sample of the robot.yaml file.

```yaml title="robot.yaml" linenums="1"
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
