#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Projectile.h"

class ControllerImage {
public:
    ControllerImage()
        : last_time_set_(false),         // Initialize flags and constants
          default_dt_(1.0 / 60.0)        // Default time step
    {
        ros::NodeHandle nh;
        image_sub_ = nh.subscribe("/game_image", 1, &ControllerImage::game_image_callback, this);
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    }

private:
    ros::Subscriber image_sub_;
    ros::Publisher joy_pub_;
    std::vector<mrsd::Projectile> prev_projectiles; // Projectiles from the last frame
    ros::Time last_time_; // Time of the last callback
    bool last_time_set_;
    const double default_dt_;

    struct Player {
        float x;
        float y;
    };

    struct Explosion {
        float x, y;
        float radius;
    };

    void game_image_callback(const sensor_msgs::ImageConstPtr& msg) {
        ros::Time now = ros::Time::now();
        double dt = default_dt_;

        if (last_time_set_) {
            dt = (now - last_time_).toSec();
        }
        last_time_ = now;
        last_time_set_ = true;

        if (dt <= 0) {
            ROS_WARN("dt is non-positive: %f", dt);
            return; // Handle zero or negative dt
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        int width = image.cols;

        Player player = find_player(image);

        int safe_spot;

        if (player.x != -1.0) { // Check if player was found
            std::pair<std::vector<mrsd::Projectile>, std::vector<Explosion>> danger = find_danger(image, dt);
            std::vector<mrsd::Projectile> projectiles = danger.first;
            std::vector<Explosion> explosions = danger.second;
            safe_spot = pick_safe_spot(projectiles, explosions, width, player);
            publish_joy_data(safe_spot, player.x);
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
            ROS_WARN("Warning, more than one player detected, picking one randomly!");
        }

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[0], center, radius);
        player.x = center.x;
        player.y = center.y;
        return player;
    }

    std::pair<std::vector<mrsd::Projectile>, std::vector<Explosion>> find_danger(cv::Mat& image, double dt) {
        std::vector<mrsd::Projectile> projectiles;
        std::vector<Explosion> explosions;
    
        cv::Mat hsv_image; // hsv is hue, saturation, and value
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Scalar lower_lim(3, 150, 150);
        cv::Scalar upper_lim(35, 255, 255);
        cv::Mat mask_image; // mask for both yellow and orange for both projectiles and explosion
        cv::inRange(hsv_image, lower_lim, upper_lim, mask_image);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < 3) { // Filter to only projectiles
                mrsd::Projectile new_projectile;
                new_projectile.x = center.x;
                new_projectile.y = center.y;
                new_projectile.vx = 0;
                new_projectile.vy = 0;
                new_projectile.predicted = 0;
                projectiles.push_back(new_projectile); // Fill in velocities after
            } else if (radius > 3) {
                explosions.push_back(Explosion{center.x, center.y, radius});
            }
        }

        if (!prev_projectiles.empty()) {
            for (auto& projectile : projectiles) {
                for (auto it = prev_projectiles.begin(); it != prev_projectiles.end();) {
                    float distance = std::sqrt(std::pow(it->x - projectile.x, 2) + std::pow(it->y - projectile.y, 2));
                    if (distance < 1.5) {
                        projectile.vx = (projectile.x - it->x) / dt;
                        projectile.vy = (projectile.y - it->y) / dt;
                        it = prev_projectiles.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
        prev_projectiles = projectiles;
        return std::make_pair(projectiles, explosions);
    }


    int pick_safe_spot(const std::vector<mrsd::Projectile>& projectiles, const std::vector<Explosion> explosions, 
    const int grid_width, const Player& player) {

        bool unsafe_cells[grid_width] = {false}; // initialize all cells to safe
        float g = 9.8;

        for (const auto& projectile : projectiles) { 
            float x = projectile.x;
            float y = projectile.y;
            float vx = projectile.vx;
            float vy = projectile.vy;
        
            double discriminant = vy * vy + 2 * g * (player.y - y);
            if (discriminant >= 0) {
                double sqrt_discriminant = std::sqrt(discriminant);
                double t1 = (-vy + sqrt_discriminant) / g;
                double t2 = (-vy - sqrt_discriminant) / g;
                double time_to_impact = (t1 > 0) ? t1 : t2;

                double impact_location = x + vx * time_to_impact;

                for (int i = std::floor(impact_location) - 6; i <= std::ceil(impact_location) + 6; i++) {
                    if (i >= 0 && i < grid_width) {
                        unsafe_cells[i] = true;
                    }
                }
            }
        }

        for (const auto& explosion : explosions) {
            float x = explosion.x;
            float radius = explosion.radius;
            for (int i = floor(x) - radius - 1; i <= ceil(x) + radius + 1; ++i) {
                if (i >= 0 && i <= grid_width) {
                    unsafe_cells[i] = true;
                }
            }
        }
    
        int safe_spot = -1;
        int low = player.x;
        int high = player.x;

        while (safe_spot == -1) {
            if (low < 0 && high >= grid_width) {
                safe_spot = player.x;
            } else if (low >= 0 && !unsafe_cells[low]) {
                safe_spot = low;
            } else if (high < grid_width && !unsafe_cells[high]) {
                safe_spot = high;
            }

            low -= 1;
            high += 1;
        }
        return safe_spot;
    }

    void publish_joy_data(int safe_spot, float player_location) {
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
