#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "assignment2_rt/msg/obstacle_info.hpp" 
#include "assignment2_rt/srv/get_avg_vel.hpp"
#include "assignment2_rt/srv/set_threshold.hpp"

#include <algorithm> 
#include <vector>
#include <deque> 
#include <limits>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        
        // --- PARAMETERS ---
        stop_dist_ = 0.50;      
        slowdown_dist_ = 1.0;   
        
        min_front_dist_ = 100.0;
        min_rear_dist_  = 100.0;
        
        // Initialize the log flag to false
        stop_log_printed_ = false;

        last_cmd_.linear.x = 0.0;
        last_cmd_.angular.z = 0.0;

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_obs_info_ = this->create_publisher<assignment2_rt::msg::ObstacleInfo>("/obstacle_info", 10);
        
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10, std::bind(&SafetyNode::cmd_callback, this, _1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&SafetyNode::scan_callback, this, _1));

        srv_avg_vel_ = this->create_service<assignment2_rt::srv::GetAvgVel>(
            "get_avg_vel", std::bind(&SafetyNode::get_avg_vel_callback, this, _1, _2));

        srv_set_thresh_ = this->create_service<assignment2_rt::srv::SetThreshold>(
            "set_threshold", std::bind(&SafetyNode::set_threshold_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Safety Node READY: Services Active.");
    }

private:
    double stop_dist_;
    double slowdown_dist_;
    double min_front_dist_;
    double min_rear_dist_;
    
    // New variable to track log state
    bool stop_log_printed_;

    geometry_msgs::msg::Twist last_cmd_; 
    std::deque<geometry_msgs::msg::Twist> velocity_history_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<assignment2_rt::msg::ObstacleInfo>::SharedPtr pub_obs_info_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    rclcpp::Service<assignment2_rt::srv::GetAvgVel>::SharedPtr srv_avg_vel_;
    rclcpp::Service<assignment2_rt::srv::SetThreshold>::SharedPtr srv_set_thresh_;

    // --- SERVICE CALLBACK 1: Average Calculation ---
    void get_avg_vel_callback(const std::shared_ptr<assignment2_rt::srv::GetAvgVel::Request> request,
                              std::shared_ptr<assignment2_rt::srv::GetAvgVel::Response> response) 
    {
        (void)request;
        
        if (velocity_history_.empty()) {
            response->avg_linear = 0.0;
            response->avg_angular = 0.0;
            return;
        }

        double sum_linear = 0.0;
        double sum_angular = 0.0;

        for (const auto& cmd : velocity_history_) {
            sum_linear += cmd.linear.x;
            sum_angular += cmd.angular.z;
        }

        response->avg_linear = sum_linear / velocity_history_.size();
        response->avg_angular = sum_angular / velocity_history_.size();
        
        RCLCPP_INFO(this->get_logger(), "Service Request: Sending Averages (Lin: %.2f, Ang: %.2f)", 
            response->avg_linear, response->avg_angular);
    }

    // --- SERVICE CALLBACK 2: Set Threshold ---
    void set_threshold_callback(const std::shared_ptr<assignment2_rt::srv::SetThreshold::Request> request,
                                std::shared_ptr<assignment2_rt::srv::SetThreshold::Response> response) 
    {
        if (request->new_threshold > 0.0) {
            stop_dist_ = request->new_threshold;
            response->success = true;
            
            // Reset the flag when the threshold changes for safety
            stop_log_printed_ = false; 
            
            RCLCPP_INFO(this->get_logger(), "Service Request: Threshold updated to %.2f m", stop_dist_);
        } else {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Service Request: Invalid Threshold");
        }
    }

    void publish_safe_cmd() {
        geometry_msgs::msg::Twist safe_cmd = last_cmd_;
        bool is_emergency_stop = false; // Local flag to determine if we are stopping now

        if (safe_cmd.linear.x > 0.0) {
            // Check FRONT
            if (min_front_dist_ < stop_dist_) {
                safe_cmd.linear.x = 0.0;
                is_emergency_stop = true;
            } else if (min_front_dist_ < slowdown_dist_) {
                double factor = (min_front_dist_ - stop_dist_) / (slowdown_dist_ - stop_dist_);
                factor = std::clamp(factor, 0.0, 1.0);
                safe_cmd.linear.x *= factor;
            }
        }
        else if (safe_cmd.linear.x < 0.0) {
            // Check REAR
            if (min_rear_dist_ < stop_dist_) {
                safe_cmd.linear.x = 0.0;
                is_emergency_stop = true;
            } else if (min_rear_dist_ < slowdown_dist_) {
                double factor = (min_rear_dist_ - stop_dist_) / (slowdown_dist_ - stop_dist_);
                factor = std::clamp(factor, 0.0, 1.0);
                safe_cmd.linear.x *= factor;
            }
        }

        // --- PRINT LOGIC HANDLED HERE ---
        if (is_emergency_stop) {
            if (!stop_log_printed_) {
                RCLCPP_INFO(this->get_logger(), "STOP! Threshold distance reached");
                stop_log_printed_ = true; // Prevents future prints while we are blocked
            }
        } else {
            // If not in emergency stop (we are slowing down or free), reset the flag
            stop_log_printed_ = false;
        }

        pub_cmd_->publish(safe_cmd);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int n = msg->ranges.size();
        int width = n / 8; 
        int start_f = (n/2) - (width/2);
        int end_f = (n/2) + (width/2);

        float temp_front = 100.0;
        for (int i = start_f; i < end_f; i++) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && r < temp_front) temp_front = r;
        }
        min_front_dist_ = temp_front;

        float temp_rear = 100.0;
        int start_r1 = 0; int end_r1 = width / 2;
        int start_r2 = n - (width / 2); int end_r2 = n;
        for (int i = start_r1; i < end_r1; i++) { if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max && msg->ranges[i] < temp_rear) temp_rear = msg->ranges[i]; }
        for (int i = start_r2; i < end_r2; i++) { if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max && msg->ranges[i] < temp_rear) temp_rear = msg->ranges[i]; }
        min_rear_dist_ = temp_rear;

        float global_min = 100.0;
        int global_idx = -1;
        for(int i=0; i<n; i++) {
            if(msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max && msg->ranges[i] < global_min) {
                global_min = msg->ranges[i];
                global_idx = i;
            }
        }
        
        std::string dir = "Unknown";
        if(global_idx != -1) {
             if (global_idx >= start_f && global_idx <= end_f) dir = "Front";
             else if (global_idx < start_f && global_idx >= width/2) dir = "Right";
             else if (global_idx > end_f && global_idx <= n - (width/2)) dir = "Left";
             else dir = "Rear";

             auto info = assignment2_rt::msg::ObstacleInfo();
             info.min_distance = global_min;
             info.direction = dir;
             info.threshold = stop_dist_;
             pub_obs_info_->publish(info);
        }

        publish_safe_cmd();
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        
        if (std::abs(msg->linear.x) > 0.001 || std::abs(msg->angular.z) > 0.001) {
            
            velocity_history_.push_back(*msg);
            
            if (velocity_history_.size() > 5) {
                velocity_history_.pop_front();
            }
        }

        publish_safe_cmd();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}