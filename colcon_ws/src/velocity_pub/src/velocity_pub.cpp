#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalPublisher::cmd_vel_callback, this, _1));
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = this->get_clock()->now();
    message.twist.linear.x = input_cmd_vel_.linear.x;
    message.twist.angular.z = input_cmd_vel_.angular.z;
//    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.twist.linear.x);
    publisher_->publish(message);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    input_cmd_vel_.linear.x = msg->linear.x;
    input_cmd_vel_.angular.z = msg->angular.z;
//    RCLCPP_INFO(this->get_logger(), "I heard: %f", msg->linear.x);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  geometry_msgs::msg::Twist input_cmd_vel_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

