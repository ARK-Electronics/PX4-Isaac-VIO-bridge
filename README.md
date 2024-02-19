## PX4 VIO using NVIDIA Isaac ROS Visual SLAM
Bridge node between [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
and PX4 using the [PX4-ROS2 DDS Bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

| Subscribed Topics | Interface |
| --------- | --------- |
| `/visual_slam/tracking/odometry` | [`nav_msgs/Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) | 
| `/visual_slam/status` | [`isaac_ros_visual_slam_interfaces/visual_slam_status`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/main/isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg) |
| `/fmu/out/sensor_combined` | [`px4_msgs/sensor_combined`](https://github.com/PX4/px4_msgs/blob/main/msg/SensorCombined.msg) |

| Published Topics | Interface |
| --------- | --------- |
| `/fmu/in/vehicle_visual_odometry` | [`px4_msgs/VehicleOdometry`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleOdometry.msg) |
| `/vio_transform/imu` | [`sensor_msgs/Imu`](https://docs.ros2.org/humble/api/sensor_msgs/msg/Imu.html) |
