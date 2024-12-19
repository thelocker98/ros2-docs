# Commander API
## What is the Commander API
The **Commander API** in **ROS 2 Navigation Stack (Nav2)** is a Python interface for programmatically controlling a robot’s navigation behavior. It simplifies interaction with the Nav2 stack by providing an intuitive, high-level API for sending navigation goals, controlling the robot, and monitoring progress.


## Installing

```bash
sudo apt install ros-humble-tf-transformations
sudo apt install python3-transforms3d
```

## Tips and Tricks
Here are a few tips and tricks I find useful
### Getting Click Coordinates
It might seem hard to get the coordinates to navigate to. If you launch the navigation package and then open a terminal and subscribe to `/clicked_point` then you can go to the top bar in rviz2 and click on publish points and then wherever you click next will get published on the `/clicked_point` topic.
```bash
ros2 topic echo /clicked_point
```
### Getting Mouse Position
Another way to get the coordinates on a map in rviz2 is to use the `Focus Camera` button on the top bar of rviz2 to display the coordinates of the mouse in the bottom left-hand corner.


## Python Script
### Setting Initial Pose
To set and initial pose use the `setInitialPose()` command to send the initial pose to the correct ROS2 node.

```python linenums="1"
q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)

initial_pose = PoseStamped()
initial_pose.header.frame_id ='map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = position_x
initial_pose.pose.position.y = position_y
initial_pose.pose.position.z = 0.0
initial_pose.pose.orientation.x = q_x
initial_pose.pose.orientation.y = q_y
initial_pose.pose.orientation.z = q_z
initial_pose.pose.orientation.w = q_w

nav.goToPose(initial_pose)
```

### Navigating to a Goal
To navigate to a goal first get the `goal_pose` then use the `goToPose` command to navigate to that specific pose.

```python linenums="1"
def create_pose_stamped(navigator: BasicNavigator, position_x: float, position_y: float, orientation_z: float):
    if math.fabs(orientation_z) > 2*math.pi:
        # convert from degrees to radian
        orientation_z = orientation_z * math.pi / 180
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id ='map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose
    
# --- Send Nav2 goal 
goal_pose = create_pose_stamped(nav, 1.72, 0.37, 1.57)
nav.goToPose(goal_pose)
# --- Wait for Nav2
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
	time.sleep(1)
# --- Print Success
print(nav.getResult())
```

### Navigating Waypoints
To navigate to a list of points use the `followWaypoints` command in the commander API and give it a array of `PoseStamped()`.

```python linenums="1"
def create_pose_stamped(navigator: BasicNavigator, position_x: float, position_y: float, orientation_z: float):
    if math.fabs(orientation_z) > 2*math.pi:
        # convert from degrees to radian
        orientation_z = orientation_z * math.pi / 180
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id ='map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose
    
# --- Send Follow waypoints
goal_pose1 = create_pose_stamped(nav, 0.226, 2.0, 180)
goal_pose2 = create_pose_stamped(nav, -0.55, 0.58, 270)
goal_pose3 = create_pose_stamped(nav, 0.59, -1.81, 0)
goal_pose4 = create_pose_stamped(nav, -2.0, -0.5, 0)
waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4]
nav.followWaypoints(waypoints)
# --- Wait for Nav2
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	print(feedback)
	time.sleep(1)
# --- Print Success
print(nav.getResult())
```


### Full Script
Here is a full example of a python script that drives the Turtlebot3 around its world:

```python title="nav2_turtlebot3_loop.py" linenums="1"
#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time, math


def create_pose_stamped(navigator: BasicNavigator, position_x: float, position_y: float, orientation_z: float):
    if math.fabs(orientation_z) > 2*math.pi:
        # convert from degrees to radian
        orientation_z = orientation_z * math.pi / 180

    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)

    pose = PoseStamped()
    pose.header.frame_id ='map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose


def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()
    
    # --- Set initial Pose
    initial_pose = create_pose_stamped(nav, -2.0, -0.5, 0)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal 
    goal_pose = create_pose_stamped(nav, 1.72, 0.37, 1.57)
    nav.goToPose(goal_pose)
    # --- Wait for Nav2
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
        time.sleep(1)
    # --- Print Success
    print(nav.getResult())
    
    # --- Send Follow waypoints
    goal_pose1 = create_pose_stamped(nav, 0.226, 2.0, 180)
    goal_pose2 = create_pose_stamped(nav, -0.55, 0.58, 270)
    goal_pose3 = create_pose_stamped(nav, 0.59, -1.81, 0)
    goal_pose4 = create_pose_stamped(nav, -2.0, -0.5, 0)
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4]
    nav.followWaypoints(waypoints)
    # --- Wait for Nav2
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
        time.sleep(1)
    # --- Print Success
    print(nav.getResult())
    
    # --- Shutdown
    nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
## More Help
For additional help visit this [site](https://docs.nav2.org/commander_api/index.html).