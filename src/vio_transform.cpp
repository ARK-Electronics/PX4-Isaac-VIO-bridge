#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp"

class VioTransform : public rclcpp::Node
{
public:
explicit VioTransform() : Node("vio_transform")
{
	_vio_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

	// QoS profile, PX4 specific
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	_vslam_sub = this->create_subscription<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", qos,
		std::bind(&VioTransform::publish, this, std::placeholders::_1));

	_vslam_status_sub = this->create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>("/visual_slam/status", qos,
		std::bind(&VioTransform::statusCallback, this, std::placeholders::_1));
}

private:
	void publish(const nav_msgs::msg::Odometry::UniquePtr msg);
	void statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg);

	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vio_pub;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vslam_sub;
	rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr _vslam_status_sub;


	uint8_t _vslam_state = 0;
};

void VioTransform::statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg)
{
	if (msg->vo_state != _vslam_state) {
		RCLCPP_INFO(get_logger(), "[VioTransform] state change: %u", msg->vo_state);
	}

	_vslam_state = msg->vo_state;
}

void VioTransform::publish(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	px4_msgs::msg::VehicleOdometry vio;

	vio.timestamp = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
	vio.timestamp_sample = vio.timestamp;

	// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
	vio.pose_frame = vio.POSE_FRAME_FRD; // same as MAV_FRAME_LOCAL_FRD


	// It's a yaw -90
	// TODO: replace with a quat rotation and try to understand why it's like this
	// Position is some fucked up coordinate system
	tf2::Vector3 position;
	position.setX(-msg->pose.pose.position.y);
	position.setY(msg->pose.pose.position.x);
	position.setZ(msg->pose.pose.position.z);

	// It's a roll 90 yaw 180
	// TODO: replace with a quat rotation and try to understand why it's like this
	// The realsense VSLAM output is left handed.
	// We convert RealSense frame (DLB) to PX4 frame (FRD) by:
	tf2::Quaternion q;
	q.setX(-msg->pose.pose.orientation.z);
	q.setY(-msg->pose.pose.orientation.y);
	q.setZ(msg->pose.pose.orientation.x);
	q.setW(msg->pose.pose.orientation.w);

	vio.position[0] = position[0];
	vio.position[1] = position[1];
	vio.position[2] = position[2];

	vio.q[0] = q[0];
	vio.q[1] = q[1];
	vio.q[2] = q[2];
	vio.q[3] = q[3];

	// TODO: still need to transform the covariances etc...

	vio.velocity_frame = vio.VELOCITY_FRAME_FRD;

	vio.velocity[0] = msg->twist.twist.linear.x;
	vio.velocity[1] = msg->twist.twist.linear.y;
	vio.velocity[2] = msg->twist.twist.linear.z;

	vio.angular_velocity[0] = msg->twist.twist.angular.x;
	vio.angular_velocity[1] = msg->twist.twist.angular.y;
	vio.angular_velocity[2] = msg->twist.twist.angular.z;

	vio.position_variance[0] = msg->pose.covariance[0];
	vio.position_variance[1] = msg->pose.covariance[7];
	vio.position_variance[2] = msg->pose.covariance[14];

	vio.orientation_variance[0] = msg->pose.covariance[21];
	vio.orientation_variance[1] = msg->pose.covariance[28];
	vio.orientation_variance[2] = msg->pose.covariance[35];

	vio.velocity_variance[0] = msg->twist.covariance[0];
	vio.velocity_variance[1] = msg->twist.covariance[7];
	vio.velocity_variance[2] = msg->twist.covariance[14];

	vio.reset_counter = 0; // TODO: look into issac_ros_vslam code to see if we can expose it
	vio.quality = _vslam_state; // 0 = unknown/unset quality

	_vio_pub->publish(vio);
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VioTransform>());
	rclcpp::shutdown();
	return 0;
}
