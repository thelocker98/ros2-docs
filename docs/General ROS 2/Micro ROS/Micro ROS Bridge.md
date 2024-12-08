# Micro ROS Bridge
## Serial Bridge

=== "docker run"
	```bash
	docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6
	```
=== "docker compose"
	```yaml title="docker-compose.yaml"
	services:
	 micro_ros_agent:
	   image: microros/micro-ros-agent:humble
	   command: serial --dev [YOUR BOARD PORT] -v6
	   network_mode: "host"
	   privileged: true
	   volumes:
	     - /dev:/dev
	     - /dev/shm:/dev/shm
	```


## TCP Bridge

=== "docker run"
	```bash
	docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO tcp4 --port 8888 -v6
	```
=== "docker compose"
	```yaml title="docker-compose.yaml"
	services:
	 micro_ros_agent:
	   image: microros/micro-ros-agent:humble
	   command: tcp4 --port 8888 -v6
	   network_mode: "host"
	   privileged: true
	   volumes:
	     - /dev:/dev
	     - /dev/shm:/dev/shm
	```


## UDP Bridge

=== "docker run"
	```bash
	docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
	```
=== "docker compose"
	```yaml title="docker-compose.yaml"
	services:
	 micro_ros_agent:
	   image: microros/micro-ros-agent:humble
	   command: udp4 --port 8888 -v6
	   network_mode: "host"
	   privileged: true
	   volumes:
	     - /dev:/dev
	     - /dev/shm:/dev/shm
	```
