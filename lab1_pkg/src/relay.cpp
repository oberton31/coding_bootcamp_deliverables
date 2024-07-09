#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class RelayNode : public rclcpp::Node {
    public:
        RelayNode() : Node("relay_node") {
            drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                "drive", 10, std::bind(&RelayNode::ackermann_callback, this, _1));
            
            // publisher for drive_relay topic
            drive_relay_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
        }
    private:

        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_relay_pub_;
        void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {

            // create new message for multuplied drive topic
            auto relay_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();

            relay_msg->drive.speed = msg->drive.speed * 3.0;
            relay_msg->drive.steering_angle = msg->drive.steering_angle * 3.0;

            drive_relay_pub_->publish(std::move(relay_msg));
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayNode>());
    rclcpp::shutdown();
    return 0;
}