#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>


class ControllerImage {
public:
    ControllerImage() {
        ros::Nodehandle nh;
        game_image_sub_ = nh.subscribe("/game_image", 10, &ControllerImage::game_image_callback, this);
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    }
private:
    struct Projectile {
        double x;
        double y;
        double vx;
        double vy;
    }

    struct Explosion {
        double radius;
        double x;
        double y;
    }

    ros::Subscriber image_sub_;
    ros::Publisher joy_pub_;
    sensor_msgs::Image msg;
    std::vector<Projectile> prev_projectiles; // projectiles from the last frame
    std::vector<Explosion> explosions;

    void game_image_callback (const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        std::vector<Projectile> projectiles;
        find_projectiles(projectiles, image, dt); // need to implement dt later
        find_explosions(explosions, image);

        // find player on map


    }
    void find_explosions(std::vector<Explosion>& explosions, cv::Mat& image) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Here, find colors that signal an explosion
        // Then, add center of explosion and radius of explosion 

    }
    void find_projectiles(std::vector<Projectile>& projectiles, cv::Mat& image, float dt) {

        // use HSV image format
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
        cv::Scalar upper_yellow = cv::Scalar(30, 255, 255);

        cv::Mat yellow_mask;
        cv::inRange(hsv_image, lower_yellow, upper_yellow, yellow_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            // Fit a minimum enclosing circle around the contour
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            // store projectiles
            projectiles.push_back({center.x, center.y, 0.0, 0.0}); // figll in velocities after
        }
        if (!prev_projectiles.empty()) {
            for (auto& projectile : projectiles) {
                for (auto it = prev_projectiles.begin(); it != prev_projectiles.end();) {
                    // Calculate distance between current projectile and previous projectile
                    float distance = std::sqrt(std::pow(it->x - projectile.x, 2) + std::pow(it->y - projectile.y, 2));
                    
                    // Check if the projectiles are within the proximity threshold (e.g., 10 pixels)
                    if (distance < 10.0) {
                        // Calculate velocities vx and vy
                        projectile.vx = (projectile.x - it->x) / dt;
                        projectile.vy = (projectile.y - it->y) / dt;

                        // Remove or mark the previous projectile as matched
                        it = prev_projectiles.erase(it); // Remove from prev_projectiles
                        // Alternatively, mark as matched (e.g., it->matched = true;)
                    } else {
                        ++it;
                    }
                }
            }
        }
        prev_projectiles = projectiles;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_image");
    ControllerImage controller;
    ros::spin();
    return 0;
}