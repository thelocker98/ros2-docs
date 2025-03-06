To operate the Robotiq 3f Gripper in ros2 it is best to use Robotiq built in support for Modbus. This allows a simple python ros2 node to control the gripper over TCP:

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from locker98_interfaces.action import Gripper  # update with your package name
from pymodbus.client import ModbusTcpClient

class gripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper_control',
            self.execute_callback
        )
        self.gripper_ip = "192.168.131.45"
        # Automatically start Modbus connection when node starts.
        self.client = ModbusTcpClient(self.gripper_ip, port=502)
        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to gripper at {self.gripper_ip}")
        else:
            self.get_logger().info(f"Modbus connection established with gripper at {self.gripper_ip}")
            self.curl_gripper(255, 50, 0)


    def destroy_node(self):
        # Close the Modbus connection when the node is killed.
        if self.client:
            self.client.close()
            self.get_logger().info("Modbus connection closed.")
        super().destroy_node()

    def activate_gripper(self):
        response = self.client.write_registers(0, [0x0100, 0x0000, 0x0000])
        if response.isError():
            self.get_logger().error(f"Activation error: {response}")
        time.sleep(0.1)

    def curl_gripper(self, speed=0xFF, force=0xFF, position=0x00FF):
        # Constrain values to one byte.
        speed = speed & 0xFF
        force = force & 0xFF
        position = position & 0xFF

        self.get_logger().info(
            f"Moving gripper to position {position} with speed {speed} and force {force}..."
        )

        speed_force = (speed << 8) | force
        response = self.client.write_registers(0, [0x0900, position, speed_force])
        if response.isError():
            self.get_logger().error(f"Operation error: {response}")
        time.sleep(0.1)

    def wait_for_action(self, start_address, poll_interval=0.1, timeout=10):
        start_time = time.time()
        while True:
            result = self.client.read_input_registers(address=start_address, count=8)
            if result.isError():
                self.get_logger().error(f"Error reading registers at {hex(start_address)}: {result}")
                time.sleep(poll_interval)
                continue
            status = result.registers[0]
            high_byte = (status >> 8) & 0xFF
            # When high_byte is no longer 0x39, assume motion is complete.
            if high_byte != 0x39:
                break
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for action to complete")
                break
            time.sleep(poll_interval)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing gripper action...")
        if not self.client:
            self.get_logger().error("Modbus client not initialized.")
            goal_handle.abort()
            return Gripper.Result(success=False, message="Modbus client not initialized")

        # Activate the gripper.
        self.get_logger().info("Activating gripper...")
        self.activate_gripper()

        # Use parameters provided in the action goal.
        position = goal_handle.request.position
        speed = goal_handle.request.speed
        force = goal_handle.request.force

        self.get_logger().info(
            f"Moving gripper to position {position} with speed {speed} and force {force}..."
        )
        self.curl_gripper(speed, force, position)
        self.wait_for_action(start_address=0, poll_interval=0.1, timeout=10)

        self.get_logger().info("Operation complete.")
        goal_handle.succeed()
        result = Gripper.Result()
        result.success = True
        result.message = "gripper operation complete"
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = gripperActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To control this ROS2 action node we can use the ros2 cli:
```bash
ros2 action send_goal /gripper_control locker98_interfaces/action/Gripper "{position: 100, speed: 50, force: 200}"
```