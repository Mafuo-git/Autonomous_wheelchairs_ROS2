#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "can_wheelchair.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WheelchairCanInterfaceNode : public rclcpp::Node
{
public:
    WheelchairCanInterfaceNode()
    : Node("wheelchair_caninterface"), count_(0)
    {
        pubCanRx_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);
        // Changed to publish speeds and direction (left_speed, right_speed, direction)
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheels_speeds_rpm", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&WheelchairCanInterfaceNode::timer_callback, this));
        subCanTx_ = this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10, 
            std::bind(&WheelchairCanInterfaceNode::can_tx_callback, this, _1));
        subCmdVel_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
            std::bind(&WheelchairCanInterfaceNode::cmd_vel_callback, this, _1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received cmd_vel: linear.x=%.2f, angular.z=%.2f",
            msg->linear.x, msg->angular.z);
        linear_x_ = msg->linear.x;
        angular_z_ = msg->angular.z;
    }
    
    void timer_callback()
    {
       
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        std::cout << "Count value: " << count_ << std::endl;
        
        can_msgs::msg::Frame canMsg;
        canMsg.header.stamp = this->now(); 
        canMsg.header.frame_id = "wheelchair_can_frame";  
        canMsg.id = CAN_WHEELCHAIR_ID_100_CONSIGNE_FRAME_ID;
        canMsg.dlc = CAN_WHEELCHAIR_ID_100_CONSIGNE_LENGTH;
        canMsg.is_rtr = false;
        canMsg.is_extended = false;
        canMsg.is_error = false;

        can_wheelchair_id_100_consigne_t canWheelchairData; 

        if (count_ == 1)
        {
            std::cout << "reset activation" << std::endl;
            canWheelchairData.activation_cons_can_axe_x = 
                can_wheelchair_id_100_consigne_activation_cons_can_axe_x_encode(0.0);
            canWheelchairData.activation_cons_can_axe_y = 
                can_wheelchair_id_100_consigne_activation_cons_can_axe_y_encode(0.0);
        }
        else
        {
            canWheelchairData.activation_cons_can_axe_x = 
                can_wheelchair_id_100_consigne_activation_cons_can_axe_x_encode(1.0);
            canWheelchairData.activation_cons_can_axe_y = 
                can_wheelchair_id_100_consigne_activation_cons_can_axe_y_encode(1.0);
        } 
        
        canWheelchairData.cons_can_axe_x = 
            can_wheelchair_id_100_consigne_cons_can_axe_x_encode(linear_x_); 
        canWheelchairData.cons_can_axe_y = 
            can_wheelchair_id_100_consigne_cons_can_axe_y_encode(angular_z_);

        uint8_t canData[8];
        can_wheelchair_id_100_consigne_pack(canData, &canWheelchairData, 
            (size_t)CAN_WHEELCHAIR_ID_100_CONSIGNE_LENGTH);
        
        for(int i = 0; i < canMsg.dlc; i++)
        {
            canMsg.data[i] = (unsigned char)canData[i];
        }
        
        pubCanRx_->publish(canMsg);
    
    }

    // New function to determine direction string
    std::string get_direction_string(bool left_dir, bool right_dir) const
    {
        if (left_dir && right_dir) return "forward";
        if (!left_dir && !right_dir) return "backward";
        if (!left_dir && right_dir) return "left";
        if (left_dir && !right_dir) return "right";
        return "unknown";
    }

    void can_tx_callback(const can_msgs::msg::Frame canMsg) const
    {
        int error = 100;
        uint8_t canData[canMsg.data.size()];
        
        for(size_t i = 0; i < canMsg.data.size(); i++)
        {
            canData[i] = (std::uint8_t)canMsg.data[i];
        }

        switch(canMsg.id)
        {
            case CAN_WHEELCHAIR_ID_111_DIAG_FRAME_ID:
            {
                can_wheelchair_id_111_diag_t canMsgSpeed;
                error = can_wheelchair_id_111_diag_unpack(&canMsgSpeed, canData, 
                    (size_t)canMsg.data.size());

                if (!error)
                {
                    // Get wheel speeds
                    double wheel_speed_right = 
                        can_wheelchair_id_111_diag_vitesse_roue_droite_decode(canMsgSpeed.vitesse_roue_droite);
                    double wheel_speed_left = 
                        can_wheelchair_id_111_diag_vitesse_roue_gauche_decode(canMsgSpeed.vitesse_roue_gauche);

                    // Get wheel directions (bits 36 & 37)
                    bool left_direction = canMsgSpeed.direction_roue_gauche;
                    bool right_direction = canMsgSpeed.direction_roue_droite;
                    
                    // Apply sign based on direction
                    float signed_left_speed = left_direction ? wheel_speed_left : -wheel_speed_left;
                    float signed_right_speed = right_direction ? wheel_speed_right : -wheel_speed_right;

                    // Determine movement direction
                    std::string direction = get_direction_string(left_direction, right_direction);

                    // Create message with signed speeds
                    auto message = std_msgs::msg::Float32MultiArray();
                    message.data = {
                        signed_left_speed,
                        signed_right_speed
                    };

                    RCLCPP_INFO(this->get_logger(), 
                        "Publishing: left_speed=%.1f, right_speed=%.1f, movement=%s",
                        message.data[0], 
                        message.data[1],
                        direction.c_str());
                    
                    publisher_->publish(message);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Error %d during CAN decoding!", error);
                }
                break;
            }
            case CAN_WHEELCHAIR_ID_110_DIAG_FRAME_ID:
            {
                RCLCPP_WARN(this->get_logger(), "CAN ID 110 received!");
                break;
            }
            default:
                break;
        }
    }
  
    // Member variables (unchanged)
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pubCanRx_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subCanTx_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;

    size_t count_;
    double linear_x_;
    double angular_z_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelchairCanInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}