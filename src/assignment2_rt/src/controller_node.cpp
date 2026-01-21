#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// Generated Service Includes
#include "assignment2_rt/srv/get_avg_vel.hpp"
#include "assignment2_rt/srv/set_threshold.hpp"

#include <iostream>
#include <chrono>
#include <thread> // For safety, although chrono is often enough

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_input", 10);
        
        // Client Initialization
        client_avg_ = this->create_client<assignment2_rt::srv::GetAvgVel>("get_avg_vel");
        client_thresh_ = this->create_client<assignment2_rt::srv::SetThreshold>("set_threshold");

        RCLCPP_INFO(this->get_logger(), "Controller Ready.");
    }

    void send_velocity(float linear, float angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        pub_vel_->publish(msg);
        
        // Differentiated log for stop vs. movement command
        if (linear == 0.0 && angular == 0.0) {
            RCLCPP_INFO(this->get_logger(), "Stop sent.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent -> Lin: %.2f, Ang: %.2f", linear, angular);
        }
    }

    // Function to call the Average Service
    void request_average() {
        if (!client_avg_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service get_avg_vel not available.");
            return;
        }

        auto request = std::make_shared<assignment2_rt::srv::GetAvgVel::Request>();
        auto result_future = client_avg_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "--- SERVICE RESPONSE ---");
            RCLCPP_INFO(this->get_logger(), "Avg Linear: %.4f", result->avg_linear);
            RCLCPP_INFO(this->get_logger(), "Avg Angular: %.4f", result->avg_angular);
            RCLCPP_INFO(this->get_logger(), "-------------------------");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error calling the service.");
        }
    }

    // Function to call the Threshold Service
    void request_new_threshold(float new_th) {
        if (!client_thresh_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service set_threshold not available.");
            return;
        }

        auto request = std::make_shared<assignment2_rt::srv::SetThreshold::Request>();
        request->new_threshold = new_th;
        
        auto result_future = client_thresh_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Threshold updated successfully!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Server refused the threshold (invalid value?).");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error calling the service.");
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Client<assignment2_rt::srv::GetAvgVel>::SharedPtr client_avg_;
    rclcpp::Client<assignment2_rt::srv::SetThreshold>::SharedPtr client_thresh_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();

    float linear_x = 0.0;
    float angular_z = 0.0;
    int choice;

    while (rclcpp::ok()) {
        std::cout << "\n--- MENU ---\n";
        std::cout << "1. Get Average Velocity (Service)\n";
        std::cout << "2. Send New Velocity Command (5s)\n";
        std::cout << "3. Change Safety Distance (Service)\n";
        std::cout << "q. Exit\n";
        std::cout << "Choice: ";
        
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Exiting...\n";
            break;
        }

        if (choice == 1) {
            node->request_average();
        }
        else if (choice == 2) {
            std::cout << "Enter Linear X: ";
            std::cin >> linear_x;
            std::cout << "Enter Angular Z: ";
            std::cin >> angular_z;

            // 1. SEND VELOCITY
            node->send_velocity(linear_x, angular_z);
            rclcpp::spin_some(node); // Force immediate sending

            // 2. WAIT 5 SECONDS
            RCLCPP_INFO(node->get_logger(), "Executing for 5 seconds...");
            rclcpp::sleep_for(5s);

            // 3. STOP THE ROBOT
            node->send_velocity(0.0, 0.0);
            rclcpp::spin_some(node); // Force immediate sending of stop command
        }
        else if (choice == 3) {
            float new_th;
            std::cout << "Enter new stop threshold (m): ";
            std::cin >> new_th;
            node->request_new_threshold(new_th);
        }
        else {
            std::cout << "Invalid choice.\n";
        }
    }

    rclcpp::shutdown();
    return 0;
}