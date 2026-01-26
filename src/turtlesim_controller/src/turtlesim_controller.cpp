#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class TurtlesimController: public rclcpp::Node
{
public:
  TurtlesimController(): Node("turtlesim_controller")
  {
    // Modificato il tipo in nav_msgs::msg::Odometry
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&TurtlesimController::topic_callback, this, _1));
      
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&TurtlesimController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if(x_ < 3.0 && x_ > -3.0){
      message.linear.x = 1.0;
      message.angular.z = 0.0;
    }
    else if (x_ >= 3.0){
      message.linear.x = 1.0;
      message.angular.z = 1.0;
    }
    else if (x_ <= -3.0){
      message.linear.x = 1.0;
      message.angular.z = -1.0;
    }
    publisher_->publish(message);
  }

  // Callback aggiornata per accettare Odometry
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // In Odometry, la posizione x si trova dentro msg->pose.pose.position.x
    x_ = msg->pose.pose.position.x;
    
    RCLCPP_INFO(this->get_logger(), "The position of the turtle is (x): '%f'", x_);
  }

  // Aggiornato il tipo della variabile membro
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist message;
  float x_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlesimController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}