# ROS 2 Subscriber

## Sample Code
Here is the code for creating a python node that subscribes to `/robot_news` at 2Hz.

=== "Python"
	``` python title="smartphone.py" linenums="1"
	#!/usr/bin/env python
	
	import rclpy
	from rclpy.node import Node
	from example_interfaces.msg import String
	
	class SmartphoneNode(Node):
	
		def __init__(self):
			super().__init__("smartphone")
			self.subscriber_ = self.create_subscription(String, 'robot_news', self.callback_robot_news, 10)
			self.get_logger().info("Smartphone has been starte.")
		
		def callback_robot_news(self, msg):
			self.get_logger().info(msg.data)
	
	def main (args=None):
		rclpy.init(args=args)
		node = SmartphoneNode()
		rclpy.spin(node)
		rclpy.shutdown()
	
	
	if __name__ == '__main__':
		main()
	```
=== "C++"
	``` python title="smartphone.cpp" linenums="1"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/msg/string.hpp"
	
	class SmartphoneNode : public rclcpp::Node
	{
	public:
		SmartphoneNode() : Node("smartphone")
		{
			subscriber_ = this->create_subscription<example_interfaces::msg::String>(
					"robot_news", 10,
					std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
			RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
		}
	
	private:
		void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
		{
			RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
		}
		rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
	};
	
	
	int main(int argc, char **argv)
	{
		rclcpp::init(argc, argv);
		auto node = std::make_shared<SmartphoneNode>();
		rclcpp::spin(node);
		rclcpp::shutdown();
		return 0;
	}
	```