// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/srv/StringMod.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture = rclcpp::Client<cpp_pubsub::srv::StringMod>::SharedFuture;


class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Parameter Declaration
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "Parameter frequency is set to 2.0 hz");

    // creating a subscriber for tthe parameter
    parameter_subscriber_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto parameterCallbackPtr =
        std::bind(&MinimalPublisher::parameter_callback, this, _1);
    parameterHandle_ = parameter_subscriber_->add_parameter_callback(
        "freq", parameterCallbackPtr);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    auto delta = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        delta, std::bind(&MinimalPublisher::timer_callback, this));

    client = this->create_client<cpp_pubsub::srv::StringMod>("modify_msg");
    RCLCPP_DEBUG(this->get_logger(), "Client created");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No service Available yet..!");
    }
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Publishing modified string: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing the modification: '%s'",
                message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
