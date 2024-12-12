# ROS 2 Publisher
## Sample Code
Here is the code for creating a node that publishes strings to `/robot_news` at 2Hz.

=== "Python"
	``` python title="robot_news_station.py" linenums="1"
	#!/usr/bin/env python
	import rclpy
	from rclpy.node import Node
	# import message type
	from example_interfaces.msg import String
	
	class RobotNewsStationNode(Node):
		def __init__(self):
			super().__init__("robot_news_station")
	
			# Create publisher with the proper message type
			self.publisher_ = self.create_publisher(String, "robot_news", 10)
			# Create timer to triger message at 2Hz
			self.timer_ = self.create_timer(0.5, self.publish_news)
			self.get_logger().info("Starting the robot news station")
	
		def publish_news(self):
			# Create massage payload
			msg = String()
			# Add 'Hello' as the data
			msg.data = "Hello"
			# Publish the data payload
			self.publisher_.publish(msg)
	
		  
	def main (args=None):
		rclpy.init(args=args)
		node = RobotNewsStationNode()
		rclpy.spin(node)
		rclpy.shutdown()
	
	
	if __name__ == '__main__':
		main()
	```

=== "C++"
	``` python title="robot_news_station.cpp" linenums="1"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/msg/string.hpp"
	
	class RobotNewsStationNode : public rclcpp::Node
	{
		public:
			RobotNewsStationNode() : Node("robot_news_station")
			{
				publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
				timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
												std::bind(&RobotNewsStationNode::publishNews, this));
				RCLCPP_INFO(this->get_logger(),"Robot News Station has been started.");
			}
	
	private:
		void publishNews()
		{
			auto msg = example_interfaces::msg::String();
			msg.data = std::string("Hi, this is") + robot_name_ + std::string(" from the Robot News Station") ;
			publisher_->publish(msg);
		}
	
		std::string robot_name_;
		rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
	};
	
	int main(int argc, char **argv)
	{
		rclcpp::init(argc, argv);
		auto node = std::make_shared<RobotNewsStationNode>();
		rclcpp::spin(node);
		rclcpp::shutdown();
		return 0;
	}
	```
