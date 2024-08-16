#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ctime> // Use <ctime> for time handling
#include "Projectile.h"
#include "Explosion.h"

class ControllerImage {
public:
    ControllerImage() 
        : last_time_set_(false), default_dt_(1.0 / 60.0)
    {
        ros::NodeHandle nh;
        image_sub_ = nh.subscribe("/game_image", 1, &ControllerImage::game_image_callback, this);
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    }

private:
    ros::Subscriber image_sub_;
    ros::Publisher joy_pub_;
    std::vector<mrsd::Projectile> prev_projectiles; // Projectiles from the last frame
    bool last_time_set_;
    std::time_t last_time_; // Use std::time_t for timekeeping
    const double default_dt_;

    struct Player {
        float x;
        float y;
    };

    void game_image_callback(const sensor_msgs::ImageConstPtr& msg) {
        double dt = default_dt_; // Start with default dt

        if (last_time_set_) {
            std::time_t now = std::time(nullptr);
            dt = std::difftime(now, last_time_); // Compute elapsed time
        }
        last_time_ = std::time(nullptr); // Update last_time
        last_time_set_ = true; // Set flag to true

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        int width = image.cols;

        std::vector<mrsd::Projectile> projectiles;

        Player player = find_player(image);

        int safe_spot;

        if (player.x != -1.0) { // Check if player was found
            find_projectiles(projectiles, image, dt);
            safe_spot = pick_safe_spot(projectiles, width, player);
            publish_joy_data(safe_spot, player.x);
            //std::vector<mrsd::Explosion> explosions;
            //find_explosions(explosions, image);

            // Pick safe spot and send message
        }
    }

    Player find_player(cv::Mat& image) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::Scalar lower_blue(100, 150, 150);
        cv::Scalar upper_blue(140, 255, 255);
        cv::Mat blue_mask;

        cv::inRange(hsv_image, lower_blue, upper_blue, blue_mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        Player player = {-1.0, -1.0}; 

        if (contours.empty()) {
            return player;
        } else if (contours.size() > 1) {
            std::cout << "Warning, more than one player detected, picking one randomly!" << std::endl;
        }

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[0], center, radius);
        player.x = center.x;
        player.y = center.y;
        return player;
    }

    void find_explosions(std::vector<mrsd::Explosion>& explosions, cv::Mat& image) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Define HSV color range for detecting explosions (example values)
        cv::Scalar lower_explosion(0, 100, 100); // Adjust based on explosion color
        cv::Scalar upper_explosion(10, 255, 255);
        cv::Mat explosion_mask;

        // cv::inRange(hsv_image, lower_explosion, upper_explosion, explosion_mask);
        // std::vector<std::vector<cv::Point>> contours;
        // cv::findContours(explosion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // for (const auto& contour : contours) {
        //     cv::Point2f center;
        //     float radius;
        //     cv::minEnclosingCircle(contour, center, radius);
        //     if (radius > 5.0) { // Filter to exclude small noise
        //         mrsd::Explosion new_explosion;
        //         new_explosion.x = center.x;
        //         new_explosion.y = center.y;
        //         new_explosion.radius = radius;
        //         explosions.push_back(new_explosion);
        //     }
        // }
    }

    void find_projectiles(std::vector<mrsd::Projectile>& projectiles, cv::Mat& image, double dt) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Define HSV color range for detecting yellow color
        cv::Scalar lower_yellow(15, 150, 150);
        cv::Scalar upper_yellow(35, 255, 255);
        cv::Mat yellow_mask;
        cv::inRange(hsv_image, lower_yellow, upper_yellow, yellow_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < 1.5) { // Filter to only projectiles
                mrsd::Projectile new_projectile;
                new_projectile.x = center.x;
                new_projectile.y = center.y;
                new_projectile.vx = 0;
                new_projectile.vy = 0;
                new_projectile.predicted = 0;
                projectiles.push_back(new_projectile); // Fill in velocities after
            }
        }

        //std::cout << "Number of projectiles detected: " << projectiles.size() << std::endl;

        if (!prev_projectiles.empty()) {
            for (auto& projectile : projectiles) {
                for (auto it = prev_projectiles.begin(); it != prev_projectiles.end();) {
                    float distance = std::sqrt(std::pow(it->x - projectile.x, 2) + std::pow(it->y - projectile.y, 2));
                    if (distance < 0.1) {
                        projectile.vx = (projectile.x - it->x) / dt;
                        projectile.vy = (projectile.y - it->y) / dt;
                        it = prev_projectiles.erase(it); // Remove from prev_projectiles
                    } else {
                        ++it;
                    }
                }
            }
        }
        prev_projectiles = projectiles;
    }

    int pick_safe_spot(const std::vector<mrsd::Projectile>& projectiles, const int grid_width, const Player& player) {
        bool unsafe_cells[grid_width] = {false}; // initialize all cells to safe
        float g = 9.8;

        // positive x, right
        // positive y, down
        for (const auto& projectile : projectiles) { 
            float x = projectile.x;
            float y = projectile.y;
            float vx = projectile.vx;
            float vy = projectile.vy;
        
            double discriminant = vy * vy + 2 * g * (player.y - y); // b^2 - 4ac where a = 0.5 * g and c = -(y - player_y)
            if (discriminant >= 0) {
                double sqrt_discriminant = std::sqrt(discriminant);
                double t1 = (-vy + sqrt_discriminant) / g;
                double t2 = (-vy - sqrt_discriminant) / g;
                double time_to_impact = (t1 > 0) ? t1 : t2;

                double impact_location = x + vx * time_to_impact;
                //std::cout << impact_location << ", ";

                // make buffer of size 10
                for (int i = std::floor(impact_location) - 8; i <= std::ceil(impact_location) + 8; i++) {
                    if (i >= 0 && i < grid_width) { // Changed condition to prevent out-of-bounds
                        unsafe_cells[i] = true;
                        //std::cout << i << ", ";
                    }
                }
            }
        }
    
        int safe_spot = -1;
        int low = player.x;
        int high = player.x;

        while (safe_spot == -1) {
            if (low < 0 && high > grid_width) {
                safe_spot = player.x;
            } else if (low >= 0 && unsafe_cells[low] == false) {
                safe_spot = low;
            } else if (high <= grid_width && unsafe_cells[high] == false) {
                safe_spot = high;
            }

            low -= 1;
            high += 1;
        }
        std::cout << safe_spot << std::endl;
        return safe_spot;
    }

    void publish_joy_data (int safe_spot, float player_location) {
        sensor_msgs::Joy joy_msg;
        joy_msg.axes.resize(1); // resize to hold one value
        float mvmt_cmd = 0.0;
        if (player_location > safe_spot) {
            mvmt_cmd = -1.0;
        } else if (player_location < safe_spot) {
            mvmt_cmd = 1.0;
        }

        joy_msg.axes[0] = mvmt_cmd;

        joy_pub_.publish(joy_msg);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_image");
    ControllerImage controller;
    ros::spin();
    return 0;
}
