# car_bridge

ROS 2 package that bridges vision-based commands and HTTP-based robot control.

It includes:
- `vision_node`: detects a red target in `/image_raw` and publishes motion commands.
- `http_status_node`: polls robot status over HTTP and forwards commands to the robot.

## Nodes

`vision_node`
- Subscribes: `/image_raw` (`sensor_msgs/msg/Image`)
- Publishes: `/car/cmd` (`std_msgs/msg/String`), `/car/cont` (`std_msgs/msg/String`, debug)
- Behavior: detects the largest red contour and publishes `F`, `B`, `L`, `R`, or `S`.
- Uses an OpenCV window (`tracked`) for visualization.

`http_status_node`
- Subscribes: `/car/cmd` (`std_msgs/msg/String`)
- Publishes: `/car/status` (`std_msgs/msg/String`), `/car/health` (`std_msgs/msg/String`)
- Behavior: polls `/status` and sends commands to `/cmd?m=<cmd>` on the robot.

## Parameters

`http_status_node`
- `robot_ip` (string, default `192.168.1.34`)
- `robot_port` (int, default `80`)
- `poll_rate_hz` (double, default `10.0`)

## Build

From a clean ROS 2 workspace:

```bash
colcon build --packages-select car_bridge
```

## Run

```bash
source install/setup.bash
ros2 run car_bridge vision_node
```

```bash
source install/setup.bash
ros2 run car_bridge http_status_node
```

## Dependencies

This package depends on ROS 2 and the following libraries:
- `rclcpp`, `std_msgs`, `sensor_msgs`
- `cv_bridge`, `image_transport`, OpenCV
- `libcurl`

Use `rosdep` to install missing dependencies for your distro.
