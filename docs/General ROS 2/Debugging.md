# Debugging
## RQT and RQT_Graph
RQT is a GUI tool to debug your ROS 2 environment and nodes. It helps visualize what is going on in the different ROS 2 programs and how they are linked to gather. It also helps us see where bugs might occur and how they might be solved and prevented.

To launch RQT we can just simply run `rqt` in our terminal.

```bash
rqt
```

RQT is made up of a bunch of different plugins. One of the most useful plugins is the `node graph` plugin found under `Plugins>Introspection>Node Graph`. This tool allows us to visualize the layout of our nodes and how they are communicating

You can also start rqt with the graph plugin enabled with one of the following two commands
```bash
# Method 1
ros2 run rqt_graph rqt_graph

# Method 2
rqt_graph
```

## Debugging ROS 2 Topics
To get the frequency of how fast a topic is being published the run this command:
```bash
ros2 topic hz /topic_to_see
```

To get information on who is publishing and listening to a topic run this command:
```bash
ros2 topic info /topic_to_see
```

To get the bandwidth of a topic can can run this command:
```bash
ros2 topic bw /topic_to_see
```


