# ROS 2 Topics
## ROS 2 Topic Overview
In ROS 2, **topics** are the communication channels used for **publish-subscribe messaging**. They allow nodes to exchange data asynchronously without direct connections. Here's how it works:

1. **Publishers**: Nodes that send messages to a topic. For example, a sensor node might publish temperature readings to a `/temperature` topic.
2. **Subscribers**: Nodes that receive messages from a topic. For instance, a logging node might subscribe to the `/temperature` topic to record data.
3. **Messages**: The data structure exchanged over topics. These are predefined formats (like integers, strings, arrays, etc.) specified in `.msg` files.
    

## Key Features:

- **Decoupled Communication**: Publishers and subscribers do not need to know about each other.
- **Topic Names**: Identified by strings (e.g., `/robot/speed`).
- **Quality of Service (QoS)**: Configurations like reliability and durability to ensure robust communication in various environments.
- **Tools**:
    - Use `ros2 topic list` to view active topics.
    - Use `ros2 topic echo` to see the data published on a topic.
    - Use `ros2 topic pub` to manually publish data for testing.

This structure supports scalable and modular robotic applications.


## Renaming a Topic at Runtime
Renaming a node at runtime is useful when their are two ROS packages with the same node name. This method will allow you to change the name of the node so that both packages can run with out interference.

The command to rename a topic is:
```bash
ros2 run {package_name} {node_name} --ros-args -r __{topic_old_name}:{topic_new_name}
```