# ROS 2 Dependencies
## Dependencies
When I comes to adding packages and libraries to ROS 2 you will have to include them in the respective programs but they will also need to be added to the build files as dependencies. Doing this is just a little bit different in python and C++ so I will show both ways. You can import or include any library but for simplicity we will just use the `example_interface` library in this example

## Python
In python you only have to add the library to the `package.xml` like shown.

```xml title="package.xml" hl_lines="10-11" linenums="1"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
	<name>my_py_pkg</name>
	<version>0.0.0</version>
	<description>TODO: Package description</description>
	<maintainer email="ros-laptop1@todo.todo">ros-laptop1</maintainer>
	<license>TODO: License declaration</license>
	
	<depend>rclpy</depend>
	<depend>example_interfaces</depend>
	
	<test_depend>ament_copyright</test_depend>
	<test_depend>ament_flake8</test_depend>
	<test_depend>ament_pep257</test_depend>
	<test_depend>python3-pytest</test_depend>
	
	<export>
		<build_type>ament_python</build_type>
	</export>
</package>
```

## C++
In C++ it is a little harder to add libraries to the build files. The first place to add the library is the `package.xml` just like in python.

```xml title="package.xml" hl_lines="12-13" linenums="1"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
	<name>my_cpp_pkg</name>
	<version>0.0.0</version>
	<description>TODO: Package description</description>
	<maintainer email="ros-laptop1@todo.todo">ros-laptop1</maintainer>
	<license>TODO: License declaration</license>

	<buildtool_depend>ament_cmake</buildtool_depend>

	<depend>rclcpp</depend>
	<depend>example_interfaces</depend>
	
	<test_depend>ament_lint_auto</test_depend>
	<test_depend>ament_lint_common</test_depend>
	
	<export>
		<build_type>ament_cmake</build_type>
	</export>

</package>
```

Next you have to add the libraries to multiple places in the `CMakeLists.txt`.
```cmake title="CMakeLists.txt" hl_lines="9-11 17 20" linenums="1"
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```