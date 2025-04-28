#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"  // Ajout pour écouter le joystick

using namespace std::chrono_literals;

class JoyToCmdVelNode : public rclcpp::Node
{
public:
    JoyToCmdVelNode()
    : Node("joy_to_cmd_vel_node")
    {
        // Créer un publisher sur /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // S'abonner au topic /joy
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVelNode::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        // Exemple : Modifier ces indices en fonction de la configuration de ta manette
        cmd_msg.linear.x = msg->axes[1] * 100.0;   // Avancer/Reculer avec le stick gauche (axe Y)
        cmd_msg.angular.z = msg->axes[0] * (-100.0);  // Rotation avec le stick gauche (axe X)

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", 
                    cmd_msg.linear.x, cmd_msg.angular.z);
                    
        publisher_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVelNode>());
    rclcpp::shutdown();
    return 0;
}

