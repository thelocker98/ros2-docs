# ROS 2 Parameters
## Overview of ROS 2 Parameters
In ROS 2, a **node parameter** is a configurable value used to adjust a node's behavior without modifying its code. Parameters are important because they provide flexibility, allowing dynamic tuning and reusability of nodes across different scenarios. For instance say you had a ROS node that took data from a USB camera and published it to a topic. You could use parameters to set the frame rate, USB port, and color depth.

## ROS 2 Parameters CLI
The following command are help full when you are debugging or tracking down a problem.

Command to list parameters
```bash
ros2 param list /node_name
```

Command to get parameter value
```bash
ros2 param get /node_name param_name
```

## Adding ROS 2 Parameters
Adding Parameters to a ROS 2 program is not that hard. Below you can see the steps.

=== "Python"
	```python title="number_publisher.py" linenums="1" hl_lines="11 12 14 15"
	#!/usr/bin/env python3
	import rclpy
	from rclpy.node import Node
	
	from example_interfaces.msg import Int64
	
	
	class NumberPublisherNode(Node):
	    def __init__(self):
	        super().__init__("number_publisher")
	        self.declare_parameter("number_to_publish", 2)
	        self.declare_parameter("publish_frequency", 1.0)
	
	        self.number_ = self.get_parameter("number_to_publish").value
	        self.publish_frequency_ = self.get_parameter("publish_frequency").value
	
	        self.number_publisher_ = self.create_publisher(Int64, "number", 10)
	        self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_number)
	        self.get_logger().info("Number publisher has been started.")
	
	    def publish_number(self):
	        msg = Int64()
	        msg.data = self.number_
	        self.number_publisher_.publish(msg)
	
	
	def main(args=None):
	    rclpy.init(args=args)
	    node = NumberPublisherNode()
	    rclpy.spin(node)
	    rclpy.shutdown()
	
	
	if __name__ == "__main__":
	    main()
	```


=== "C++"

	```cpp title="number_publisher.cpp" linenums="1"  hl_lines="9 10 12 13"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/msg/int64.hpp"
	
	class NumberPublisherNode : public rclcpp::Node
	{
	public:
	    NumberPublisherNode() : Node("number_publisher")
	    {
	        this->declare_parameter("number_to_publish", 2);
	        this->declare_parameter("publish_frequency", 1.0);
	
	        number_ = this->get_parameter("number_to_publish").as_int();
	        double publish_frequency = this->get_parameter("publish_frequency").as_double();
	
	        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
	        number_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / publish_frequency)),
	                                                std::bind(&NumberPublisherNode::publishNumber, this));
	        RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
	    }
	
	private:
	    void publishNumber()
	    {
	        auto msg = example_interfaces::msg::Int64();
	        msg.data = number_;
	        number_publisher_->publish(msg);
	    }
	
	    int number_;
	    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
	    rclcpp::TimerBase::SharedPtr number_timer_;
	};
	
	int main(int argc, char **argv)
	{
	    rclcpp::init(argc, argv);
	    auto node = std::make_shared<NumberPublisherNode>();
	    rclcpp::spin(node);
	    rclcpp::shutdown();
	    return 0;
	}
	```