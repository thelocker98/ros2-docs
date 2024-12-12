# ROS 2 Rosbag
## What is ROS 2 Rosbag
**Rosbag** is a tool for recording and replaying topic data. It is important because it enables debugging, analysis, and testing by capturing live data streams and replaying them for offline evaluation or simulation.

### The command to record a topic 
```bash
ros2 bag record /{topic to record} -o {filename}
```

### The command to play back a topic
```bash
ros2 bag play {filename}
```

### The command to get information about a recording
```
ros2 bag info {filename}
```