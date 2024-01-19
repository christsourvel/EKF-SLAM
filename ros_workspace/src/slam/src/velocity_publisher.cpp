#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <random>


class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("velocity_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&VelocityPublisher::publishRandomVelocity, this));
    }

private:
    void publishRandomVelocity() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = randomDouble(-2.0, 2.0);  // Random vx
        msg.linear.y = randomDouble(-2.0, 2.0);  // Random vy
        msg.angular.z = randomDouble(-1.0, 1.0); // Random omega

        // Print the velocities
        std::cout << "Publishing velocities - "
                  << "vx: " << msg.linear.x << ", "
                  << "vy: " << msg.linear.y << ", "
                  << "omega: " << msg.angular.z << std::endl;

        publisher_->publish(msg);
    }

    double randomDouble(double min, double max) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}