#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm> 
#include <vector>

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        
        // --- PARAMETRI ---
        stop_dist_ = 0.50;      // Distanza di STOP (50 cm)
        slowdown_dist_ = 1.0;   // Distanza di Rallentamento (1 metro)
        
        min_front_dist_ = 100.0; 
        min_rear_dist_  = 100.0; // Distanza ostacolo posteriore

        // Inizializza memoria comando
        last_cmd_.linear.x = 0.0;
        last_cmd_.angular.z = 0.0;

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10, std::bind(&SafetyNode::cmd_callback, this, _1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&SafetyNode::scan_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Safety Node COMPLETO: Fronte + Retro attivi.");
    }

private:
    double stop_dist_;
    double slowdown_dist_;
    double min_front_dist_;
    double min_rear_dist_;
    
    geometry_msgs::msg::Twist last_cmd_; 
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    // --- LOGICA DI SICUREZZA BIDIREZIONALE ---
    void publish_safe_cmd() {
        geometry_msgs::msg::Twist safe_cmd = last_cmd_;

        // --- CASO 1: Tenta di andare AVANTI ---
        if (safe_cmd.linear.x > 0.0) {
            
            if (min_front_dist_ < stop_dist_) {
                safe_cmd.linear.x = 0.0; // BLOCCA AVANTI
            }
            else if (min_front_dist_ < slowdown_dist_) {
                // Rallentamento progressivo avanti
                double factor = (min_front_dist_ - stop_dist_) / (slowdown_dist_ - stop_dist_);
                
                // Fix Indentazione: separare gli if
                if (factor < 0.0) factor = 0.0;
                if (factor > 1.0) factor = 1.0;
                
                safe_cmd.linear.x *= factor;
                
                // Limite massimo velocità in avvicinamento
                if (safe_cmd.linear.x > 0.15) safe_cmd.linear.x = 0.15;
            }
        }

        // --- CASO 2: Tenta di andare INDIETRO ---
        else if (safe_cmd.linear.x < 0.0) {
            
            if (min_rear_dist_ < stop_dist_) {
                safe_cmd.linear.x = 0.0; // BLOCCA INDIETRO
            }
            else if (min_rear_dist_ < slowdown_dist_) {
                // Rallentamento progressivo indietro
                double factor = (min_rear_dist_ - stop_dist_) / (slowdown_dist_ - stop_dist_);
                
                // Fix Indentazione: separare gli if
                if (factor < 0.0) factor = 0.0;
                if (factor > 1.0) factor = 1.0;
                
                safe_cmd.linear.x *= factor;
                
                // In retro siamo più cauti, max -0.15
                if (safe_cmd.linear.x < -0.15) safe_cmd.linear.x = -0.15;
            }
        }

        // Nota: safe_cmd.angular.z non viene mai toccato, quindi puoi sempre girare.
        pub_cmd_->publish(safe_cmd);
    }

    // --- LETTURA SENSORI (FRONTE E RETRO) ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int n = msg->ranges.size();
        
        // --- 1. CALCOLO FRONTE (Centro dell'array) ---
        int width = n / 8; 
        int start_f = (n/2) - (width/2);
        int end_f = (n/2) + (width/2);

        float temp_front = 100.0;
        for (int i = start_f; i < end_f; i++) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && r < temp_front) {
                temp_front = r;
            }
        }
        min_front_dist_ = temp_front;

        // --- 2. CALCOLO RETRO (Inizio e Fine dell'array) ---
        // Il retro è diviso in due pezzi: indice [0..X] e indice [Y..360]
        int start_r1 = 0;
        int end_r1 = width / 2;
        
        int start_r2 = n - (width / 2);
        int end_r2 = n;

        float temp_rear = 100.0;
        
        // Scansiona prima parte del retro (es. 0° a 20°)
        for (int i = start_r1; i < end_r1; i++) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && r < temp_rear) temp_rear = r;
        }
        // Scansiona seconda parte del retro (es. 340° a 360°)
        for (int i = start_r2; i < end_r2; i++) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && r < temp_rear) temp_rear = r;
        }
        min_rear_dist_ = temp_rear;

        // Esegui controllo sicurezza immediato
        publish_safe_cmd();
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        publish_safe_cmd();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}