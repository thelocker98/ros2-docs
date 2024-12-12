# ROS 2 Interfaces
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


## Variables in ROS 2 Interfaces
You can find more information at the [ROS 2 Interface Documentation](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html).

| Type name | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](https://design.ros2.org/articles/mapping_dds_types.html) |
| --------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| bool      | bool                                                                  | builtins.bool                                                               | boolean                                                             |
| byte      | uint8_t                                                               | builtins.bytes*                                                             | octet                                                               |
| char      | char                                                                  | builtins.int*                                                               | char                                                                |
| float32   | float                                                                 | builtins.float*                                                             | float                                                               |
| float64   | double                                                                | builtins.float*                                                             | double                                                              |
| int8      | int8_t                                                                | builtins.int*                                                               | octet                                                               |
| uint8     | uint8_t                                                               | builtins.int*                                                               | octet                                                               |
| int16     | int16_t                                                               | builtins.int*                                                               | short                                                               |
| uint16    | uint16_t                                                              | builtins.int*                                                               | unsigned short                                                      |
| int32     | int32_t                                                               | builtins.int*                                                               | long                                                                |
| uint32    | uint32_t                                                              | builtins.int*                                                               | unsigned long                                                       |
| int64     | int64_t                                                               | builtins.int*                                                               | long long                                                           |
| uint64    | uint64_t                                                              | builtins.int*                                                               | unsigned long long                                                  |
| string    | std::string                                                           | builtins.str                                                                | string                                                              |
| wstring   | std::u16string                                                        | builtins.str                                                                | wstring                                                             |

_Every built-in-type can be used to define arrays:_

|Type name|[C++](https://design.ros2.org/articles/generated_interfaces_cpp.html)|[Python](https://design.ros2.org/articles/generated_interfaces_python.html)|[DDS type](https://design.ros2.org/articles/mapping_dds_types.html)|
|---|---|---|---|
|static array|std::array<T, N>|builtins.list*|T[N]|
|unbounded dynamic array|std::vector|builtins.list|sequence|
|bounded dynamic array|custom_class<T, N>|builtins.list*|sequence<T, N>|
|bounded string|std::string|builtins.str*|string|

All types that are more permissive than their ROS definition enforce the ROS constraints in range and length by software.