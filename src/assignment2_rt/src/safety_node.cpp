#include "rclcpp/rclcpp.hpp"
#include "assignment2_rt/msg/obstacle_info.hpp"
#include "assignment2_rt/srv/get_avg_vel.hpp"
#include "assignment2_rt/srv/set_threshold.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("safety_node");
    
    RCLCPP_INFO(node->get_logger(), "Safety Node Avviato!");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}