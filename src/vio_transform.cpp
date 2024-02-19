#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp"

class VioTransform : public rclcpp::Node
{
public:
explicit VioTransform() : Node("vio_transform")
{
	// QoS profile, PX4 specific
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	// PX4 formatted VIO publisher
	_vio_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

	// ROS2 formatted IMU publisher
	_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/vio_transform/imu", 10);

	// Isaac ROS VSLAM subscriptions
	_vslam_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", qos,
						std::bind(&VioTransform::odometryCallback, this, std::placeholders::_1));

	_vslam_status_sub = this->create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>("/visual_slam/status", qos,
						std::bind(&VioTransform::statusCallback, this, std::placeholders::_1));

	// FC IMU subscription
	_fc_imu_sub = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
						std::bind(&VioTransform::sensorCombinedCallback, this, std::placeholders::_1));
}

private:
	// Subscription callbacks
	void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
	void statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg);
	void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg);

	// Publishers
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vio_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

	// Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vslam_odom_sub;
	rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr _vslam_status_sub;
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr _fc_imu_sub;
	uint8_t _vslam_state = 0;
};

void VioTransform::sensorCombinedCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg)
{
	auto fc_imu_acc = tf2::Vector3();
	fc_imu_acc.setX(msg->accelerometer_m_s2[0]);
	fc_imu_acc.setY(msg->accelerometer_m_s2[1]);
	fc_imu_acc.setZ(msg->accelerometer_m_s2[2]);

	auto fc_imu_gyro = tf2::Vector3();
	fc_imu_gyro.setX(msg->gyro_rad[0]);
	fc_imu_gyro.setY(msg->gyro_rad[1]);
	fc_imu_gyro.setZ(msg->gyro_rad[2]);

	auto accel = fc_imu_acc;
	auto gyro = fc_imu_gyro;

	auto imu_msg = sensor_msgs::msg::Imu();
	imu_msg.header.stamp = get_clock()->now();
	imu_msg.linear_acceleration.x = accel[0];
	imu_msg.linear_acceleration.y = accel[1];
	imu_msg.linear_acceleration.z = accel[2];
	imu_msg.angular_velocity.x = gyro[0];
	imu_msg.angular_velocity.y = gyro[1];
	imu_msg.angular_velocity.z = gyro[2];
	imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	imu_msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
	imu_msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
	_imu_pub->publish(imu_msg);
}

void VioTransform::statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg)
{
	if (msg->vo_state != _vslam_state) {
		RCLCPP_INFO(get_logger(), "[VioTransform] state change: %u", msg->vo_state);
	}

	_vslam_state = msg->vo_state;
}

void VioTransform::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	tf2::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
	tf2::Vector3 angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
	tf2::Vector3 position_variance(msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14]);
	tf2::Vector3 orientation_variance(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]);
	tf2::Vector3 velocity_variance(msg->twist.covariance[0], msg->twist.covariance[7], msg->twist.covariance[14]);

	// NOTE: isaac_ros_vslam w/ realsense publishes Odometry in FLU world frame AKA NWU (north west up)
	tf2::Quaternion rotation;
	rotation.setRPY(M_PI, 0.0, 0.0);

	position = tf2::quatRotate(rotation, position);
	quaternion = rotation * quaternion * rotation.inverse();
	velocity = tf2::quatRotate(rotation, velocity);
	angular_velocity = tf2::quatRotate(rotation, angular_velocity);
	position_variance = tf2::quatRotate(rotation, position_variance);
	orientation_variance = tf2::quatRotate(rotation, orientation_variance);
	velocity_variance = tf2::quatRotate(rotation, velocity_variance);

	// Fill the message
	px4_msgs::msg::VehicleOdometry vio;

	vio.timestamp = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
	vio.timestamp_sample = vio.timestamp;

	vio.pose_frame = vio.POSE_FRAME_FRD;

	vio.q[0] = quaternion.getW();
	vio.q[1] = quaternion.getX();
	vio.q[2] = quaternion.getY();
	vio.q[3] = quaternion.getZ();

	vio.position[0] = position.getX();
	vio.position[1] = position.getY();
	vio.position[2] = position.getZ();

	// NOTE: body frame velocities
	vio.velocity_frame = vio.VELOCITY_FRAME_BODY_FRD;
	vio.velocity[0] = velocity.getX();
	vio.velocity[1] = velocity.getY();
	vio.velocity[2] = velocity.getZ();

	vio.angular_velocity[0] = angular_velocity.getX();
	vio.angular_velocity[1] = angular_velocity.getY();
	vio.angular_velocity[2] = angular_velocity.getZ();

	vio.position_variance[0] = position_variance.getX();
	vio.position_variance[1] = position_variance.getY();
	vio.position_variance[2] = position_variance.getZ();

	vio.orientation_variance[0] = orientation_variance.getX();
	vio.orientation_variance[1] = orientation_variance.getY();
	vio.orientation_variance[2] = orientation_variance.getZ();

	vio.velocity_variance[0] = velocity_variance.getX();
	vio.velocity_variance[1] = velocity_variance.getY();
	vio.velocity_variance[2] = velocity_variance.getZ();

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
