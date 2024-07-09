#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class TalkerNode : public rclcpp::Node
{
    public:
        TalkerNode(): Node("talker_node")
        {
            // create parameters v and d
            this->declare_parameter<double>("v", 0.0);
            this->declare_parameter<double>("d", 0.0);

            // set v_ and d_ to 0.0 
            this->get_parameter("v", v_);
            this->get_parameter("d", d_);
            // Create publisher to topic "drive"
            drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
            
            // create timer that calls publish_ackermann_drive to be called every 10 ms
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&TalkerNode::publish_ackermann_drive, this));
        }
    private:
        // initializing variables, created shared pointers for timer_ and drive_pub_
        double v_;
        double d_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

        void publish_ackermann_drive() {

            // create a messaage
            auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
            ackermann_msg->drive.speed = v_;
            ackermann_msg->drive.steering_angle = d_;

            // publish message
            drive_pub_->publish(std::move(ackermann_msg));

        }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}

