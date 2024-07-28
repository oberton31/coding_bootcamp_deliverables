#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <vector>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher() : Node("waypoint_publisher") {

        // csv file path parameter
        this->declare_parameter<std::string>("csv_file_path", "path/to/your/waypoints.csv");
        std::string file_path;
        this->get_parameter("csv_file_path", file_path);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", 10);
        load_waypoints(file_path);

        // Timer to periodically publish markers
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { this->publish_markers();});
    }

private:
    void load_waypoints(const std::string &file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;
            geometry_msgs::msg::Point point;

            std::getline(ss, item, ','); point.x = std::stof(item);
            std::getline(ss, item, ','); point.y = std::stof(item);
            std::getline(ss, item, ','); point.z = std::stof(item);

            // Skip orientation, not needed for vis
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            waypoints_.push_back(point);
        }
    }

    void publish_markers() {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        for (size_t idx = 0; idx < waypoints_.size(); ++idx)
        {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "waypoints";
            marker.id = static_cast<int>(idx);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = waypoints_[idx];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Point> waypoints_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
