## PX4 VIO using NVIDIA Isaac ROS Visual SLAM
Bridge node between [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
and PX4 using the [PX4-ROS2 DDS Bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

**Subscribed**
| Topic | Interface |
| --------- | --------- |
| `/visual_slam/tracking/odometry` | [`nav_msgs/Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) | 

**Published**
| Topic | Interface |
| --------- | --------- |
| `/fmu/in/vehicle_visual_odometry` | [`px4_msgs/VehicleOdometry`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleOdometry.msg) |

