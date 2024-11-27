# Setup
## Installing VSCode
first install vscode using snap
```bash
sudo snap install code --classic
```
then add the `ROS` extension from Microsoft.

## Installing Colcon
To install colcon first run the sudo apt install command.
``` bash
sudo apt install python3-colcon-common-extensions
```

then add the colcon autocomplete to the `.bashrc` file.
``` bash
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc
```


# Setting up the ROS2 Package Workspace
first create the folder for the package usual it is called `packagename_ws`where `_ws` stands for then you enter the directory and create another directory called `src` then run the `colcon build` command.
```bash
colcon build
```
next go into the `install` folder and add the `setup.bash` source command to your `~/.bashrc`
```bash
echo "source /home/username/package_ws/install/setup.bash" >> ~/.bashrc
```

## Creating a Python Packages
#### Setting Up
Go to the workspace folder that was created in the last step and open the `src` folder. In this folder run this command.
```bash
ros2 pkg create {package_name} --build-type ament_python --dependencies rclpy
```

add your code in to the folder with the `__init__.py`. This is usually in the folder `src/{package_name}/{package_name}`

#### Programming
create a python file for your first node.
`my_first_node.py`
```python
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

#### Building
go the the `{package_name}_ws` folder with the `src` and `install` folder in it. Then run the `colcon build` command.
```bash
colcon build
```

next run the `source ~/.bashrc` command to reload the workspace source file and the other source files.

#### Simplifying Build (specific to python)
since we are using python and it is interpreted we can simplify the build process when working buy using the symlink command to avoid building.
```bash
colcon build --symlink-install
```

then just like before run the `source ~/.bashrc` command to refresh everything and after that you will never have to worry about building or refreshing again.



## Creating a C++ Packages
Go to the workspace folder that was created in the last step and open the `src` folder. In this folder run this command.
```bash
ros2 pkg create {package_name} --build-type ament_cmake --dependencies rclcpp
```

![Image title](img/ros2_nodes.png)
