dd# ROS 2 Nodes
## Node Layout
Nodes are the different ROS topics. in the below picture the nodes are in blue.
![[ros2_nodes.png|900]]

## Getting Node info
To get information about a ROS 2 node you can use the ros2 cli to explore the nodes.

The command `ros2 node list` shows a list of all the nodes avalible. It can be handy to make sure the nodes are all discovered and working properly. 
```bash
ros2 node list
```

Their is also a ROS 2 command for getting information about an individual node.
```bash
ros2 node info {/node_name}
```

## Renaming a Node at Runtime
Renaming a node at runtime is useful when their are two ROS packages with the same node name. This method will allow you to change the name of the node so that both packages can run with out interference.

The command to rename a node is:
```bash
ros2 run {package_name} {node_name} --ros-args -r __node:{new_name}
```