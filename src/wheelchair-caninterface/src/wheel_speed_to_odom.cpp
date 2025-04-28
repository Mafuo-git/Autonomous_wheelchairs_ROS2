#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

class WheelSpeedToOdometry : public rclcpp::Node {
public:
    WheelSpeedToOdometry() : Node("wheel_speed_to_odometry"), x_(0.0), y_(0.0), theta_(0.0) {
        // Declare and get parameters for wheel radius, wheel base, and frame IDs
        this->declare_parameter("wheel_radius", 0.29);
        this->declare_parameter("wheel_base", 0.585);
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("child_frame_id", "base_footprint");

        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);

        // Subscribe to the topic publishing wheel RPMs
        wheel_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheels_speeds_rpm", 10,
            std::bind(&WheelSpeedToOdometry::wheel_speed_callback, this, std::placeholders::_1));

        // Publisher for the odometry data
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Initialize the TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Store the initial timestamp
        last_time_ = this->now();
    }

private:
    void wheel_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received incomplete wheel speeds!");
            return;
        }

        // Extract wheel RPMs from the message
        float left_rpm = msg->data[0];
        float right_rpm = msg->data[1];

        // Convert RPM to radians per second
        float left_rad_s = (left_rpm * 2.0 * M_PI) / 60.0;
        float right_rad_s = (right_rpm * 2.0 * M_PI) / 60.0;

        // Compute linear and angular velocities
        float v = (wheel_radius_ / 2.0) * (right_rad_s + left_rad_s);
        float omega = (wheel_radius_ / wheel_base_) * (right_rad_s - left_rad_s);

        // Compute time difference since last update
        rclcpp::Time current_time = this->now();
        float dt = (current_time - last_time_).seconds();

        // Update robot's pose based on calculated velocities
        float delta_x = v * cos(theta_) * dt;
        float delta_y = v * sin(theta_) * dt;
        float delta_theta = omega * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Convert orientation to quaternion for TF/odometry message
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Publish the transform from odom to base_footprint
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = frame_id_;
        odom_tf.child_frame_id = child_frame_id_;
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);

        // Fill the odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = frame_id_;
        odom_msg.child_frame_id = child_frame_id_;
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = omega;

        // Publish odometry
        odometry_publisher_->publish(odom_msg);

        // Update last timestamp
        last_time_ = current_time;
    }

    // Parameters
    float wheel_radius_;
    float wheel_base_;
    std::string frame_id_;
    std::string child_frame_id_;

    // Robot's current pose state
    float x_, y_, theta_;
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// Main entry point for the node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelSpeedToOdometry>());
    rclcpp::shutdown();
    return 0;
}
