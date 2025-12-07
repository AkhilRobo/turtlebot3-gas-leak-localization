#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class BayesGasLocalizer : public rclcpp::Node
{
public:
    BayesGasLocalizer() : Node("bayes_gas_localizer")
    {
        map_size_ = 20.0;     // 20x20 meters
        resolution_ = 0.5;    // 0.5m per cell
        grid_dim_ = static_cast<int>(map_size_ / resolution_);
        int num_cells = grid_dim_ * grid_dim_;

        //[0.000625,0,.000625,...] for 40x40=1600 cells
        belief_grid_.assign(num_cells, 1.0 / num_cells);  

        robot_pose_received_ = false;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&BayesGasLocalizer::odom_callback, this, _1));
            
        gas_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gas_reading", 10, std::bind(&BayesGasLocalizer::gas_callback, this, _1));

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/leak_probability_map", 1);
        
        RCLCPP_INFO(this->get_logger(), "Bayes Filter Initialized. Grid Size: %dx%d", grid_dim_, grid_dim_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        robot_pose_received_ = true;
    }

    void gas_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!robot_pose_received_) return;

        double z = msg->data; 
        double total_probability = 0.0;

        for (int i = 0; i < grid_dim_; ++i) { // y-axis index
            for (int j = 0; j < grid_dim_; ++j) { // x-axis index
                
                int index = i * grid_dim_ + j;

                // Get world coordinates of this cell center
                double cx = (j * resolution_) - (map_size_ / 2.0);
                double cy = (i * resolution_) - (map_size_ / 2.0);

                // Distance from robot to this specific cell
                double d = std::sqrt(std::pow(current_pose_.position.x - cx, 2) + 
                                     std::pow(current_pose_.position.y - cy, 2));

                double expected_z = 100.0 * std::exp(-(std::pow(d, 2)) / (2 * std::pow(1.5, 2)));

                double likelihood = std::exp(-(std::pow(z - expected_z, 2)) / 10.0);

                // Update Belief (Posterior = Likelihood * Prior)
                belief_grid_[index] *= likelihood;
                
                total_probability += belief_grid_[index];
            }
        }

        auto grid_msg = nav_msgs::msg::OccupancyGrid();
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "odom";
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_dim_;
        grid_msg.info.height = grid_dim_;
        grid_msg.info.origin.position.x = -(map_size_ / 2.0);
        grid_msg.info.origin.position.y = -(map_size_ / 2.0);
        grid_msg.data.resize(grid_dim_ * grid_dim_);

        double max_prob = 0.0;

        if (total_probability > 0) {
            for (auto &p : belief_grid_) {
                p /= total_probability;
                if (p > max_prob) max_prob = p;
            }
        }

        
        for (size_t k = 0; k < belief_grid_.size(); ++k) {
            if (max_prob > 0) {
                int val = static_cast<int>((belief_grid_[k] / max_prob) * 100.0);
                grid_msg.data[k] = std::min(100, std::max(0, val));
            } else {
                grid_msg.data[k] = 0;
            }
        }

        grid_pub_->publish(grid_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gas_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    std::vector<double> belief_grid_;
    geometry_msgs::msg::Pose current_pose_;
    bool robot_pose_received_;
    
    double map_size_;
    double resolution_;
    int grid_dim_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BayesGasLocalizer>());
    rclcpp::shutdown();
    return 0;
}