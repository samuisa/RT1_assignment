#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // Pubblica su /cmd_vel_input (che verrà intercettato dal SafetyNode)
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_input", 10);
        
        RCLCPP_INFO(this->get_logger(), "Controller Pronto. Comandi: [i]Avanti [,]Indietro [j]Sinistra [l]Destra [k]Stop");
        RCLCPP_INFO(this->get_logger(), "NOTA: Scrivi la lettera e premi INVIO.");
    }

    void send_velocity(float linear, float angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        pub_vel_->publish(msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();

    char input;
    bool running = true;

    // Loop Utente
    while (rclcpp::ok() && running) {
        std::cout << "Comando > ";
        std::cin >> input; // Nota: Blocca l'esecuzione finché non premi Invio

        switch (input) {
            case 'i': node->send_velocity(0.5, 0.0); break;  // Avanti
            case ',': node->send_velocity(-0.5, 0.0); break; // Indietro
            case 'j': node->send_velocity(0.0, 0.5); break;  // Sinistra
            case 'l': node->send_velocity(0.0, -0.5); break; // Destra
            case 'k': node->send_velocity(0.0, 0.0); break;  // Stop
            case 'q': running = false; break;
            default:  
                // Se premi altro, ferma per sicurezza
                node->send_velocity(0.0, 0.0); 
                break;
        }
        
        // Permette a ROS di processare brevemente la pubblicazione
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}