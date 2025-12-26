#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2_rt/srv/set_threshold.hpp"
#include "assignment2_rt/srv/get_avg_vel.hpp"

#include <iostream>
#include <limits>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // Publisher per inviare comandi di velocità al robot
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client per il servizio di cambio soglia
        client_threshold_ = this->create_client<assignment2_rt::srv::SetThreshold>("set_threshold");

        // Client per il servizio di media velocità
        client_avg_ = this->create_client<assignment2_rt::srv::GetAvgVel>("get_avg_vel");
        
        RCLCPP_INFO(this->get_logger(), "Controller Node Pronto. Usa l'interfaccia testuale.");
    }

    // Funzione per inviare velocità
    void send_velocity(float linear, float angular) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear;
        msg.angular.z = angular;
        pub_vel_->publish(msg);
    }

    // Funzione per chiamare il servizio SetThreshold
    void call_set_threshold() {
        if (!client_threshold_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio set_threshold non disponibile.");
            return;
        }

        float new_th;
        std::cout << "Inserisci nuova soglia (metri): ";
        std::cin >> new_th;

        auto request = std::make_shared<assignment2_rt::srv::SetThreshold::Request>();
        request->new_threshold = new_th;

        // Chiamata asincrona
        auto result_future = client_threshold_->async_send_request(request);
        
        // Attendiamo il risultato (in un nodo semplice come questo va bene fare wait)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Soglia aggiornata con successo!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Errore nella chiamata al servizio.");
        }
    }

    // Funzione per chiamare il servizio GetAvgVel
    void call_get_avg_vel() {
        if (!client_avg_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Servizio get_avg_vel non disponibile.");
            return;
        }

        auto request = std::make_shared<assignment2_rt::srv::GetAvgVel::Request>();
        auto result_future = client_avg_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto result = result_future.get();
            std::cout << "\n--- MEDIA VELOCITA' (Ultimi 5 comandi) ---" << std::endl;
            std::cout << "Lineare: " << result->avg_linear << std::endl;
            std::cout << "Angolare: " << result->avg_angular << std::endl;
            std::cout << "------------------------------------------" << std::endl;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Errore nella chiamata al servizio.");
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Client<assignment2_rt::srv::SetThreshold>::SharedPtr client_threshold_;
    rclcpp::Client<assignment2_rt::srv::GetAvgVel>::SharedPtr client_avg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();

    char input;
    bool running = true;

    while (rclcpp::ok() && running) {
        std::cout << "\n=== CONTROLLER MENU ===" << std::endl;
        std::cout << "i: Avanti" << std::endl;
        std::cout << "k: Stop" << std::endl;
        std::cout << "j: Sinistra" << std::endl;
        std::cout << "l: Destra" << std::endl;
        std::cout << ",: Indietro" << std::endl;
        std::cout << "t: Imposta Soglia Sicurezza (Service)" << std::endl;
        std::cout << "g: Ottieni Media Velocità (Service)" << std::endl;
        std::cout << "q: Esci" << std::endl;
        std::cout << "Comando: ";
        std::cin >> input;

        // Svuota buffer input nel caso l'utente scriva più caratteri
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        switch (input) {
            case 'i': node->send_velocity(0.5, 0.0); break;
            case ',': node->send_velocity(-0.5, 0.0); break;
            case 'j': node->send_velocity(0.0, 0.5); break;
            case 'l': node->send_velocity(0.0, -0.5); break;
            case 'k': node->send_velocity(0.0, 0.0); break;
            case 't': node->call_set_threshold(); break;
            case 'g': node->call_get_avg_vel(); break;
            case 'q': running = false; break;
            default: std::cout << "Comando non valido." << std::endl; break;
        }
        
        // Importante: permette a ROS di processare eventuali messaggi in background
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}