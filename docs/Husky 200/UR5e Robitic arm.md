The Robotic arm on the Husky A200 from Clearpath robotics can be controlled using moveit2 in ROS 2 but due to the lack of compute on the Husky A200 it is more stable to use the UR Dashboard server. This allows simple commands to be sent through a socket on port 29999. You can learn more about the UR Dashboard server at this [link](https://forum.universal-robots.com/t/could-not-understand-after-loading-program-using-dashboard/27104/5) and [this one](https://s3-eu-west-1.amazonaws.com/ur-support-site/42728/DashboardServer_e-Series_2022.pdf). Here is some example code for triggering programs using the dashboard server.

```python
#!/usr/bin/env python3
import socket

# Define robot IP and dashboard port
ROBOT_IP = "192.168.131.40"
DASHBOARD_PORT = 29999

def send_command(command):
    """Open a connection, send the command, print and return the response."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, DASHBOARD_PORT))
        # The robot sends an initial welcome message upon connecting.
        welcome = s.recv(1024).decode('utf-8').strip()
        print("Welcome message:", welcome)
        # Send the command (ending with newline)
        s.sendall((command + "\n").encode('utf-8'))
        # Receive the response
        response = s.recv(1024).decode('utf-8').strip()
        print(f"Response for '{command}': {response}")
        return response

# Power On
response = send_command("power on")
print(f"Power On Response: {response}")

# Release Brake
response = send_command("brake release")
print(f"Brake Release Response: {response}")

# Load the program "grap_water_bottle"
response = send_command("load grap_water_bottle.urp")
print(f"Power On Response: {response}")

# Play the program
response = send_command("play")
print(f"Power On Response: {response}")