# ROS 2 Services

## Remapping Service Topics
Remapping service names can come in handy when packages have duplicate service names or you want to run the same service twice.
```bash
ros2 run your_package your_node --ros-args -r old_name:=new_name
```

## Debugging a Service
Sometimes you do not know what type of message a service takes. To find the right type use the `ros2 service type` command:
```bash
ros2 service type /service_topic
```
## Programming a Service
ROS 2, **services** enable synchronous communication between nodes. A service consists of a **request** and **response** model: one node (client) sends a request, and another node (service server) processes it and returns a response.

Here is the code for making a simple ROS 2 Service

=== "Python"
	```python title="add_two_ints_server.py" linenums="1"
	#!/usr/bin/env python
	import rclpy
	from rclpy.node import Node
	from example_interfaces.srv import AddTwoInts
	
	class AddTwoIntsServerNode(Node):
	    def __init__(self):
	        super().__init__("add_two_ints_server")
	        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints);
	        self.get_logger().info("Add Two Ints Server has been started.")
	
	    def callback_add_two_ints(self, request, response):
	        response.sum = request.a + request.b
	        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
	        return response
	
	
	def main (args=None):
	    rclpy.init(args=args)
	    node = AddTwoIntsServerNode()
	    rclpy.spin(node)
	    rclpy.shutdown()
	
	if __name__ == '__main__':
	    main()
	```
=== "C++"
	```cpp title="add_two_ints_server.cpp" linenums="1"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/srv/add_two_ints.hpp"
	
	using std::placeholders::_1;
	using std::placeholders::_2;
	
	class AddTwoIntServerNode : public rclcpp::Node
	{
	public:
	    AddTwoIntServerNode() : Node("add_two_ints_server")
	    {
	        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
	            "add_two_ints",
	            std::bind(&AddTwoIntServerNode::callbackAddTwoInts, this, _1, _2));
	        
	        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
	    }
	
	private:
	
	    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
	                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
	    {
	        response->sum = request->a + request->b;
	        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
	    }
	
	    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
	};
	
	int main(int argc, char **argv)
	{
	    rclcpp::init(argc, argv);
	    auto node = std::make_shared<AddTwoIntServerNode>();
	    rclcpp::spin(node);
	    rclcpp::shutdown();
	    return 0;
	}
	```

## Programming a Client
ROS 2, **Clients** are nodes or components that initiate the service call, while the service itself is implemented by the server node. This interaction is commonly used for tasks requiring immediate feedback, such as retrieving or setting specific values.

### Manual Making A Service Call
Sometime it is handy to make a ROS 2 service call manually. To do this use the `ros2 service call` command:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 1}"
```

### Without Object Orientated Programming
This program creates a client without using object orientated programming.
=== "Python"
	```python title="add_two_ints_client_no_oop.py" linenums="1"
	#!/usr/bin/env python3
	import rclpy
	from rclpy.node import Node
	
	from example_interfaces.srv import AddTwoInts
	
	def main(args=None):
	    rclpy.init(args=args)
	    node = Node("add_two_ints_client_no_oop")
	
	    client = node.create_client(AddTwoInts, "add_two_ints")
	    while not client.wait_for_service(1.0):
	        node.get_logger().warn("Waiting for Server Add Two Ints...")
	
	    request = AddTwoInts.Request()
	    request.a = 3
	    request.b = 8
	
	    future = client.call_async(request)
	    rclpy.spin_until_future_complete(node, future)
	
	    try:
	        response = future.result()
	        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
	    except Exception as e:
	        node.get_Togger().error("Service call failed %r" % (e,))
	
	    rclpy.shutdown()
	
	if __name__ == "__main__":
	    main()
	```
=== "C++"
	```cpp title="add_two_ints_client_no_oop.cpp" linenums="1"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/srv/add_two_ints.hpp"

	int main(int argc, char **argv)
	{
	    rclcpp::init(argc, argv);
	    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");
	    
	    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
	
	    while(!client->wait_for_service(std::chrono::seconds(1)))
	    {
	        RCLCPP_WARN(node->get_logger(), "Waiting for the server to be up");
	    }
	
	    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
	    request->a = 3;
	    request->b = 8;
	
	    auto future = client->async_send_request(request);
	
	    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
	    {
	        RCLCPP_INFO(node->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)future.get()->sum);
	    }
	    else{
	        RCLCPP_ERROR(node->get_logger(), "Error while calling service");
	    }
	
	    rclcpp::shutdown();
	    return 0;
	}
	```
### With Object Orientated Programming
This program creates a client with object orientated programming.
=== "Python"
	```python title="add_two_ints_client.py" linenums="1"
	#!/usr/bin/env python3
	import rclpy
	from rclpy.node import Node
	from functools import partial
	
	from example_interfaces.srv import AddTwoInts
	
	
	class AddTwoIntsClientNode(Node):
	    def __init__(self):
	        super().__init__("add_two_ints_client")
	        self.call_add_two_ints_server(6,7)
	
	    def call_add_two_ints_server(self, a, b):
	        client = self.create_client(AddTwoInts, "add_two_ints")
	        while not client.wait_for_service(1.0):
	            self.get_logger().warn("Waiting For Server Add Ints...")
	        
	        request = AddTwoInts.Request()
	        request.a = a
	        request.b = b
	        
	        future = client.call_async(request)
	        
	        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
	
	    def callback_call_add_two_ints(self, future, a, b):
	        try:
	            response = future.result()
	            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
	        except Exception as e:
	            self.get_Togger().error("Service call failed %r" % (e,))
	
	
	def main (args=None):
	    rclpy.init(args=args)
	    node = AddTwoIntsClientNode()
	    rclpy.spin(node)
	    rclpy.shutdown()
	
	if __name__ == '__main__':
	    main()
	```
=== "C++"
	```cpp title="add_two_ints_client.cpp" linenums="1"
	#include "rclcpp/rclcpp.hpp"
	#include "example_interfaces/srv/add_two_ints.hpp"
	
	
	class AddTwoIntsClientNode : public rclcpp::Node
	{
	public:
	    AddTwoIntsClientNode() : Node("add_two_ints_client")
	    {
	        thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 10, 4));
	    }
	
	    void callAddTwoIntsService(int a, int b)
	    {
	        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
	        while(!client->wait_for_service(std::chrono::seconds(1)))
	        {
	            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up");
	        }
	
	        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
	        request->a = a;
	        request->b = b;
	
	        auto future = client->async_send_request(request);
	        try
	        {
	            auto response = future.get();
	            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response->sum);
	        }
	        catch(const std::exception &e)
	        {
	            RCLCPP_ERROR(this->get_logger(), "Service call failed");
	        }
	        
	
	    }
	
	private:
	    std::thread thread1_;
	};
	
	int main(int argc, char **argv)
	{
	    rclcpp::init(argc, argv);
	    auto node = std::make_shared<AddTwoIntsClientNode>();
	    rclcpp::spin(node);
	    rclcpp::shutdown();
	    return 0;
	}
	```