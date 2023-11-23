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
		std::bind(&VioTransform::odometryCallback, this, std::placeholders::_1));

	_vslam_status_sub = this->create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>("/visual_slam/status", qos,
		std::bind(&VioTransform::statusCallback, this, std::placeholders::_1));
}

private:
	void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
	void statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg);

	// NOTE: isaac_ros_vslam w/ realsense publishes Odometry in FLU world frame AKA NWU (north west up)
	void NWU_to_NED_position(tf2::Vector3& position);
	void NWU_to_NED_orientation(tf2::Quaternion& quat);

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

void VioTransform::NWU_to_NED_position(tf2::Vector3& position)
{
	tf2::Quaternion NED_NWU_Q;
	NED_NWU_Q.setRPY(M_PI, 0.0, 0.0);
	// rotate the position vector from NWU to NED
	position = tf2::quatRotate(NED_NWU_Q, position);
}

void VioTransform::NWU_to_NED_orientation(tf2::Quaternion& quat)
{
	tf2::Quaternion NED_NWU_Q;
	NED_NWU_Q.setRPY(M_PI, 0.0, 0.0);
	// rotate quaterion from NWU into NED
	quat = NED_NWU_Q * quat * NED_NWU_Q.inverse();
}

void VioTransform::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	px4_msgs::msg::VehicleOdometry vio;

	vio.timestamp = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
	vio.timestamp_sample = vio.timestamp;

	vio.pose_frame = vio.POSE_FRAME_NED;

	tf2::Vector3 position;
	position.setX(msg->pose.pose.position.x);
	position.setY(msg->pose.pose.position.y);
	position.setZ(msg->pose.pose.position.z);

	tf2::Quaternion quat;
	quat.setW(msg->pose.pose.orientation.w);
	quat.setX(msg->pose.pose.orientation.x);
	quat.setY(msg->pose.pose.orientation.y);
	quat.setZ(msg->pose.pose.orientation.z);

	NWU_to_NED_position(position);
	NWU_to_NED_orientation(quat);

	vio.position[0] = position[0];
	vio.position[1] = position[1];
	vio.position[2] = position[2];

	vio.q[0] = quat.getW();
	vio.q[1] = quat.getX();
	vio.q[2] = quat.getY();
	vio.q[3] = quat.getZ();

	// TODO: still need to transform the covariances etc...

	vio.velocity_frame = vio.VELOCITY_FRAME_NED;

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
