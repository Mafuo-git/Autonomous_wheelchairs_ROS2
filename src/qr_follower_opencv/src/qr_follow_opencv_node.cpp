#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <geometry_msgs/msg/twist.hpp>

class QRFollower : public rclcpp::Node
{
public:
  QRFollower() : Node("qr_follower")
  {
    // Image transport
    auto node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});
    it_ = std::make_shared<image_transport::ImageTransport>(node);

    // Subscribe to camera image
    image_sub_ = it_->subscribe("/camera/color/image_raw", 1,
      std::bind(&QRFollower::imageCallback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Control parameters
    linear_gain_ = 2.0;
    angular_gain_ = 2.0;
    min_linear_percent_ = 20.0;
    deadzone_pixels_ = 20;
    max_angular_percent_ = 70.0;

    RCLCPP_INFO(this->get_logger(), "QR Follower OpenCV node initialized");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try {
      // Convert ROS image to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat frame = cv_ptr->image;

      // QR code detection
      cv::QRCodeDetector qr_detector;
      std::vector<cv::Point> points;
      std::string decoded = qr_detector.detectAndDecode(frame, points);

      if (!decoded.empty() && points.size() == 4) {
        // Center of QR code
        cv::Point2f center(0.f, 0.f);
        for (const auto& p : points)
          center += cv::Point2f(p.x, p.y);
        center *= 0.25f;

        // Image center
        double image_center_u = frame.cols / 2.0;
        double image_center_v = frame.rows / 2.0;

        // Errors
        double error_u = center.x - image_center_u;
        double error_v = center.y - image_center_v;

        geometry_msgs::msg::Twist cmd;

        // Angular (rotation)
        if (fabs(error_u) > deadzone_pixels_) {
          double angular_percent = angular_gain_ * error_u;
          angular_percent = std::clamp(angular_percent, -max_angular_percent_, max_angular_percent_);
          cmd.angular.z = angular_percent;
        }

        // Linear (avance)
        if (fabs(error_v) < image_center_v * 0.3) {
          double linear_percent = linear_gain_ * (100.0 - (100.0 * fabs(error_v) / image_center_v));
          linear_percent = std::max(linear_percent, min_linear_percent_);
          cmd.linear.x = linear_percent;
        }

        RCLCPP_INFO(this->get_logger(),
          "QR at (%.1f,%.1f) Cmd: lin.x=%.1f%% ang.z=%.1f%%",
          center.x, center.y, cmd.linear.x, cmd.angular.z);

        cmd_vel_pub_->publish(cmd);
        return;
      }

      // No QR code detected
      RCLCPP_INFO(this->get_logger(), "No QR code detected - stopping (0%%)");
      auto stop_cmd = geometry_msgs::msg::Twist();
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(stop_cmd);
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
  }

  // Member variables
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Control parameters
  double linear_gain_;
  double angular_gain_;
  double min_linear_percent_;
  int deadzone_pixels_;
  double max_angular_percent_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QRFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
