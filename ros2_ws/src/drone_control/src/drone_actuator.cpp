#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



class DroneActuator : public rclcpp::Node
{ 
    public: 
        DroneActuator() : Node("drone_actuator")
        { 
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>( 
                "cmd_vel", 10, 
                std::bind(&DroneActuator::move_drone, this, std::placeholders::_1) 
            );

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10);
        }

    private:
        void move_drone(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Moving drone - Linear: [x: %.2f, y: %.2f, z: %.2f], Angular: [x: %.2f, y: %.2f, z: %.2f]", 
                msg->linear.x, msg->linear.y, msg->linear.z);
            // Here you would add the code to actually move the drone using the received velocities.
            publisher_->publish(*msg);
             

        }
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
      
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneActuator>());
    rclcpp::shutdown();
    return 0;
}
