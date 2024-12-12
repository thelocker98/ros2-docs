# ROS 2 Custom Interfaces

## Creating Custom Interface
In ROS 2, a **custom interface** defines unique message, service, or action types tailored to specific application needs. It is important because it enables precise communication between nodes, ensuring data is structured and interpreted correctly for specialized robotics tasks.

### Making Package
To create a custom interface it is best to make a dedicated package for all the interfaces that are needed in that project. Go to the workspace and navigate to the `src` folder. 
```bash
ros2 pkg create {robot_name}_interfaces
```

Once the package is create open the package folder with visual studio code and delete the `include` directory and the `src` directory. Next open the `packages.xml` file and delete all the lines that have the `<depend>{somthing}</depend>`. Finally, add the following highlighted lines.

```xml title="packages.xml" linenums="1" hl_lines="12-14"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ros-laptop1@todo.todo">ros-laptop1</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Now that the `packages.xml` is one the `CMakeLists.txt` file needs to be changed. Go through and delete everything so it looks like this and add the lines that are highlighted.
```cmake title="CMakeLists.txt" linenums="1" hl_lines="10 12-15 17"
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/{service_name}.msg"
  "srv/{service_name}.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

Lines 13 and 14 are the custom interfaces that you created and want to build. Look at the next two sections to learn about how to create these files with the message and server configuration.

### Creating a Custom Message
To create a custom message navigate to the package that was created in the [[#Making Package]] section. Here we will create a folder in this package called `msg` and inside this folder we will put all of are message configs. For example lets create a message for a color sensor. This message might contain the the red, green, and blue value from the sensor.
```xml title="ColorSensor.msg" linenums="1"
# you can add comments to your message configs like this
int64 red # comments can be added here also
int64 green
int64 blue
```

Now all you have to do add it to the `CMakeLists.txt` file and run `colcon build`.

### Creating a Custom Service
To create a custom service navigate to the package that was created in the [[#Making Package]] section. Here we will create a folder in this package called `srv` and inside this folder we will put all of are service configs. For example lets create a service for a rectangular area calculator. This service might have a length and width for a request and as a response just an area.
```xml title="CalculateArea.srv" linenums="1"
# you can add comments to your message configs like this
# the --- seperates the request from the response
float64 length
float64 width
---
float64 area
```

Now all you have to do add it to the `CMakeLists.txt` file and run `colcon build`.

### Using a Custom Interface
To use a custom interface in a Python or C++ program import it or include it just like normal. If you have problems with vscode not recognizing it in the C++ programs you will have to edit the `.vscode/c_cpp_properties.json` file and add it to the include path.
```json title="c_cpp_properties.json" linenums="1" hl_lines="11"
{
    "configurations": [
        {
            "browse": {
                "databaseFilename": "${default}",
                "limitSymbolsToIncludedHeaders": false
            },
            "includePath": [
                "/opt/ros/humble/include/**",
                "/usr/include/**",
                "~/some/folder/{workspace_name}_ws/install/{robot_name}_interfaces/include/**"
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

