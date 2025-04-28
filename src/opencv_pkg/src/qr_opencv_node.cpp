#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>

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
    linear_gain_ = 0.75;
    angular_gain_ = 0.75;
    Ki_ = 0.0;
    Kd_ = 0.1;
    min_linear_percent_ = 20.0;
    max_linear_percent_ = 50.0;
    deadzone_pixels_ = 20;
    max_angular_percent_ = 70.0;

    target_dist = 2.0; //target distance in meters

    focal_lenght_px = 1400.0;
    qr_real_size_mm = 250.0;

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
      bool found = qr_detector.detect(frame, points);

      if (found && points.size() == 4) {
        // Center of QR code
        cv::Point2f center(0.f, 0.f);
        for (const auto& p : points)
          center += cv::Point2f(p.x, p.y);
        center *= 0.25f;

        // Image center
        double image_center_u = frame.cols / 2.0;
        double image_center_v = frame.rows / 2.0;

        double width = cv::norm(points[0] - points[1]);
        double height = cv::norm(points[1] - points[2]);
        qr_img_size_px = std::max(width,height);
        double dist = (focal_lenght_px*qr_real_size_mm)/qr_img_size_px; //in milimeters
        dist = dist/1000.0; // in meters
        
        
        // Errors
        double error_u = center.x - image_center_u;
        double error_lin = dist - target_dist;
        integral_lin_ += error_lin;
        integral_ang_ += error_u;
        double derivate_ang_ = error_u - prev_error_ang_;
        double derivate_lin_ = error_lin - prev_error_lin_;
        prev_error_lin_ = error_lin;
        prev_error_ang_ = error_u;

        geometry_msgs::msg::Twist cmd;

        // Angular (rotation)
        if (fabs(error_u) > deadzone_pixels_) {
          double angular_percent = angular_gain_ * error_u + integral_ang_ * Ki_ + derivate_ang_ * Kd_;
          //double angular_percent = angular_gain_ * error_u;
          angular_percent = std::clamp(angular_percent, -max_angular_percent_, max_angular_percent_);
          cmd.angular.z = angular_percent;
        }

        // Linear (avance)
        if (error_lin > 0.1) {
          double linear_percent = 100 * (linear_gain_ * fabs(error_lin) + Ki_ * integral_lin_ + Kd_ * derivate_lin_) / target_dist;
          //double linear_percent = 100 * linear_gain_ * (fabs(error_lin) / target_dist);
          linear_percent = std::clamp(linear_percent, min_linear_percent_,max_linear_percent_);
          cmd.linear.x = linear_percent;
        }
        else
        {
          cmd.linear.x = 0.0;
        }

        RCLCPP_INFO(this->get_logger(),
          "QR at (%.1f,%.1f), dist=%.2f Cmd: lin.x=%.1f%% ang.z=%.1f%%",
          center.x, center.y, dist, cmd.linear.x, cmd.angular.z);

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
  double Kd_;
  double Ki_;
  double Kfu_;
  
  
  double min_linear_percent_;
  double max_linear_percent_;
  int deadzone_pixels_;
  double max_angular_percent_;
  
  double target_dist;
  double focal_lenght_px;
  double qr_real_size_mm;
  double qr_img_size_px;

  double integral_lin_ = 0.0;
  double integral_ang_ = 0.0;
  
  double prev_error_lin_ = 0.0;
  double prev_error_ang_ = 0.0;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QRFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
