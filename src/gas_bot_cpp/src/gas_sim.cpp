#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <random>

using std::placeholders::_1;

class GasSimulator : public rclcpp::Node
{
public:
    GasSimulator() : Node("gas_simulator")
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32>("/gas_reading", 10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GasSimulator::odom_callback, this, _1));

        source_x_ = 2.0;
        source_y_ = 1.0;
        source_strength_ = 100.0;
        sigma_ = 1.5; 

        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::normal_distribution<>(0.0, 0.5); // Mean 0, StdDev 0.5

        RCLCPP_INFO(this->get_logger(), "Gas Simulator Started. Leak at (%f, %f)", source_x_, source_y_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double rx = msg->pose.pose.position.x;
        double ry = msg->pose.pose.position.y;

        double dist = std::sqrt(std::pow(rx - source_x_, 2) + std::pow(ry - source_y_, 2));

        //Gaussian Plume Equation
        double reading = source_strength_ * std::exp(-(std::pow(dist, 2)) / (2 * std::pow(sigma_, 2)));

        // Add Noise
        double noise = dist_(gen_);
        double final_reading = std::max(0.0, reading + noise);

        auto message = std_msgs::msg::Float32();
        message.data = final_reading;
        pub_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    
    double source_x_, source_y_, source_strength_, sigma_;
    std::mt19937 gen_;
    std::normal_distribution<> dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GasSimulator>());
    rclcpp::shutdown();
    return 0;
}