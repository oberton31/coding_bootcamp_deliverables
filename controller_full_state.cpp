#include "ros/ros.h"
#include <game/GameState.h>
#include <game/Projectile.h>
#include "sensor_msgs/Joy.h"
#include <cmath>

class ControllerFullState {
public:
    ControllerFullState() {
        ros::NodeHandle nh;
        game_state_sub_ = nh.subscribe("/game_state", 10, &ControllerFullState::game_state_callback, this);
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    }


private:
    ros::Subscriber game_state_sub_;
    ros::Publisher joy_pub_;
    game::GameState msg;

    void game_state_callback(const game::GameStateConstPtr& msg) {
        auto game_state = *msg;
        std::vector<game::Player> players = game_state.players;
        if (!players.empty()) {
            int player_location = players[0].x;
            int safe_spot_ = pick_safe_spot(game_state);
            //ROS_INFO("Safe spot chosen: %d", safe_spot_);
            publish_joy_data(safe_spot_, player_location);
        } 

    }

    int pick_safe_spot (const game::GameState& game_state) {
        std::vector<game::Projectile> projectiles = game_state.projectiles;
        int grid_size = game_state.w;

        bool unsafe_cells[grid_size] = {false}; // initialize all cells to safe

        float g = 9.8;

        for (const auto& projectile : projectiles) {
            float x = projectile.x;
            float y = projectile.y;
            float vx = projectile.vx;
            float vy = projectile.vy;
        
            // using 2-d kinematics, calculate x position where projectile will hit ground
            double discriminant = vy * vy + 2 * g * y; // b^2 - 4ac where a = -0.5 * g and c = y

            if (discriminant < 0) {
                continue;
            }

            double sqrt_discriminant = sqrt(discriminant);
            double t1 = (-vy + sqrt_discriminant) / (-g);
            double t2 = (-vy - sqrt_discriminant) / (-g);
            double time_to_impact = (t1 > 0) ? t1 : t2;

            double impact_location = x + vx * time_to_impact;

            // make buffer of size 10
            for (int i = floor(impact_location) - 8; i <= ceil(impact_location) + 8; i++) {
                if (i >= 0 && i <= grid_size) {
                    unsafe_cells[i] = true;
                }
            }
        }

        // ensure that we dont run into any explosions
        std::vector<game::Explosion> explosions = game_state.explosions;

        for (const auto& explosion: explosions) {
            int x = explosion.x;
            for (int i = floor(x) - 8; i <= ceil(x) + 8; i++) {
                if (i >= 0 && i <= grid_size) {
                    unsafe_cells[i] = true;
                }
            }
        }

        std::vector<game::Player> players = game_state.players;
        int player_location = players[0].x;

        int safe_spot = -1;
        int low = player_location;
        int high = player_location;

        while (safe_spot == -1) {
            //ROS_INFO("low: %d, high: %d", low, high);
            if (low < 0 && high > grid_size) {
                safe_spot = player_location;
            } else if (low >= 0 && unsafe_cells[low] == false) {
                safe_spot = low;
            } else if (high <= grid_size && unsafe_cells[high] == false) {
                safe_spot = high;
            }

            low -= 1;
            high += 1;
        }
        return safe_spot;
    }

    void publish_joy_data (int safe_spot, int player_location) {
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
    // Initialize ROS node
    ros::init(argc, argv, "controller_full_state");
    
    // Create an instance of ControllerFullState
    ControllerFullState controller;
    
    // Spin the ROS node
    ros::spin();
    
    return 0;
}