#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trial_interfaces/msg/numbers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisherSubscriber : public rclcpp::Node
{
  public:
    MinimalPublisherSubscriber()
    : Node("minimal_publishersubscriber"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic", 10);
      publisher_numbers = this->create_publisher<trial_interfaces::msg::Numbers>("topic3", 10);

      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisherSubscriber::timer_callback, this));

      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "topic2", 10, std::bind(&MinimalPublisherSubscriber::topic_callback, this, _1));

      subscription_numbers = this->create_subscription<trial_interfaces::msg::Numbers>(
      "topic4", 10, std::bind(&MinimalPublisherSubscriber::topic_callbacknumber, this, _1));

    }
  
  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Int32();
      message.data = count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      publisher_->publish(message);

      auto messageNumbers = trial_interfaces::msg::Numbers();
      messageNumbers.number1 = count_++;
      messageNumbers.number2 = count_*2.0;
      messageNumbers.number3.data = count_+3;
      RCLCPP_INFO(this->get_logger(), "Publishing Numbers: '%d,%f,%d'", messageNumbers.number1, messageNumbers.number2, messageNumbers.number3.data);
      publisher_numbers->publish(messageNumbers);
    }
    void topic_callback(const std_msgs::msg::Int32 & msg) const
    {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }
    void topic_callbacknumber(const trial_interfaces::msg::Numbers & msg) const
    {
    RCLCPP_INFO(this->get_logger(), "I heard Number: '%d,%f,%d'", msg.number1, msg.number2, msg.number3.data);
      
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Publisher<trial_interfaces::msg::Numbers>::SharedPtr publisher_numbers;
    rclcpp::Subscription<trial_interfaces::msg::Numbers>::SharedPtr subscription_numbers;
    int count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisherSubscriber>());
  rclcpp::shutdown();

  return 0;
}