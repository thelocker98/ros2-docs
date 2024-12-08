# Message Types
## What is a Message Type
In ROS 2 (Robot Operating System 2), a `msg` type refers to a message definition that contains data structures used for communication between nodes. These messages are the building blocks for publishing and subscribing to topics, which enable nodes to exchange information with each other. Think of a message as a container that holds specific types and amounts of data, like a digital package being sent between nodes in a ROS 2 nodes. Each type of message corresponds to a particular set of data fields and their data types (e.g., `int8`, `float32`, `String`). When you define a message, you're specifying the structure of the data that will be sent or received by nodes. This allows for more organized and efficient communication between nodes.
## How to find them
When creating a publisher you have to figure out what data type it is going to publish. To find examples of the different data types in ROS 2 you can run a special command.

```bash  { .annotate }
ros2 interface show example_interfaces/msg/String #(1)
```

1. In this case it was the `String` type that I used but it could have been any other type. To get a list use `Tab`before typing `String` for a autocomplete list of options.

## Including Message Types
Once you have found the message type you will need to make sure that it is imported. This can be pretty easily in both C++ and Python.

Below you can see that we use the `String` as the message type. This example works for other types as well of messages as well. You can just swap `String` for whatever other data type that is available. 
=== "Python"
	```python
	from example_interfaces.msg import String 
	```

=== "C++"
	``` c++
	#include "example_interfaces/msg/string.hpp"
	```



