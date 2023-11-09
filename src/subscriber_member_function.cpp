/**
 * @file subscriber_member_function.cpp
 * @author Akashkumar parmar (akasparm@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

/**
 * @brief Subscriber class
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Function to call the topic and to fetch the message
   * from it
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    // Logging information when listener node is active
    RCLCPP_INFO(this->get_logger(), "I am the listener node: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}