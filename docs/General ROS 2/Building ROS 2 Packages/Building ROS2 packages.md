# Building ROS2 packages
## Setup

### Setting up the ROS2 Package Workspace
First create the folder for the package usual it is called `packagename_ws`where `_ws` stands for then you enter the directory, and create, another directory called `src` then run the `colcon build` command.
```bash
colcon build
```
next go into the `install` folder and add the `setup.bash` source command to your `~/.bashrc`
```bash
echo "source /home/username/package_ws/install/setup.bash" >> ~/.bashrc
```

## Creating a Python Packages
### Setting Up
Go to the workspace folder that was created in the last step and open the `src` folder. In this folder run this command.
```bash
ros2 pkg create {package_name} --build-type ament_python --dependencies rclpy
```

add your code in to the folder with the `__init__.py`. This is usually in the folder `src/{package_name}/{package_name}`

### Programming
create a python file for your first node.

```python title="my_first_node.py" linenums="1"
#!/usr/bin/env python
import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__('first_node')
        self.get_logger().info('Hello from ROS2')


def main (args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now add it to the setup.py file in the `entry_points` section.

it is setup with the name of the ROS2 command first then the equals followed by the package name, python file name that is in the same folder as the `__init__.py`, and finally the function name to call, usual `main`.
```python
    entry_points={
        'console_scripts': [
            "test_node = husky_test.my_first_node:main"
        ],
    },
```

The final step is building the script and testing it.

### Building
go the the `{package_name}_ws` folder with the `src` and `install` folder in it. Then run the `colcon build` command.
```bash
colcon build
```

next run the `source ~/.bashrc` command to reload the workspace source file and the other source files.

To help improve the build speed we can also use `--packages-select` to only build a specific package.
```bash
colcon build --packages-select {package_name}
```


### Simplifying Build (specific to python)
Since we are using python, and it is interpreted we can simplify the build process when working by using the symlink command to avoid building.
```bash
colcon build --symlink-install
```

Then just like before run the `source ~/.bashrc` command to refresh everything and after that you will never have to worry about building or refreshing again.



## Creating a C++ Packages
Go to the workspace folder that was created in the last step and open the `src` folder. In this folder run this command.
```bash
ros2 pkg create {package_name} --build-type ament_cmake --dependencies rclcpp
```


next create a file in `{package_name}/src`and call it `node_name.cpp`

```cpp title="node_name.cpp" linenums="1"
#include "rclcpp/rclcpp.hpp"  
  
class MyNode : public rclcpp::Node  
{  
public:  
   MyNode() : Node("cpp_test"), counter_(0)  
   {  
       RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");  
  
       timer_ = this->create_wall_timer(std::chrono::milliseconds(500),  
                                        std::bind(&MyNode::timerCallback, this));  
   }  
  
private:  
   void timerCallback()  
   {  
       counter_++;  
       RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);  
   }  
  
   rclcpp::TimerBase::SharedPtr timer_;  
   int counter_;  
};  
  
int main(int argc, char **argv)  
{  
   rclcpp::init(argc, argv);  
   auto node = std::make_shared<MyNode>();  
   rclcpp::spin(node);  
   rclcpp::shutdown();  
   return 0;  
}
```

Next open the program menu on VSCode using `CTRL + SHIFT + P`. Select the `C/C++: Edit Configurations (JSON)`. Make Sure the `c_cpp_properties.json` file has the `"/opt/ros/humble/include/**"` include

```json hl_lines="9" linenums="1"
{
	"configurations": [
		{
			"browse": {
				"databaseFilename": "${default}",
				"limitSymbolsToIncludedHeaders": false
			},
			"includePath": [
				"/opt/ros/humble/include/**",
				"/usr/include/**"
			],
			"name": "ROS",
			"intelliSenseMode": "gcc-x64",
			"compilerPath": "/usr/bin/gcc",
			"cStandard": "gnu11",
			"cppStandard": "c++14"
		}
	],
	"version": 4
}
```