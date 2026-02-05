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

## Requirements

- ROS 2 Humble on Ubuntu 22.04.
- A camera publishing `sensor_msgs/msg/Image` on `/image_raw`.

Example camera bringup (USB webcam):

```bash
ros2 run v4l2_camera v4l2_camera_node
```

If your camera publishes on a different topic, remap it when launching, for example:

```bash
ros2 run car_bridge vision_node --ros-args -r /image_raw:=/camera/image_raw
```

## Build

From a clean ROS 2 workspace:

```bash
colcon build --packages-select car_bridge
```

## Run Nodes

```bash
source install/setup.bash
ros2 run car_bridge vision_node
```

```bash
source install/setup.bash
ros2 run car_bridge http_status_node
```

## Run Launch

```bash
ros2 launch car_bridge car.launch.py 
```

## Dependencies

This package depends on ROS 2 and the following libraries:
- `rclcpp`, `std_msgs`, `sensor_msgs`
- `cv_bridge`, `image_transport`, OpenCV
- `libcurl`

Use `rosdep` to install missing dependencies for your distro.

## Vision Tuning Notes

`vision_node` uses fixed HSV thresholds to detect red and simple area thresholds to decide motion:

- Red mask thresholds (HSV):
  - Low range: `H 0-10`, `S 120-255`, `V 80-255`
  - High range: `H 170-179`, `S 120-255`, `V 80-255`
- Area thresholds:
  - Publish debug area on `/car/cont` when `area > 1000`
  - Accept target when `area > 500`
  - Move `F` when `area < 8000`
  - Move `B` when `area >= 15000`

If detection is unstable, adjust lighting or move the target closer/farther. To change the thresholds, update the constants in `src/vision_node.cpp`.
