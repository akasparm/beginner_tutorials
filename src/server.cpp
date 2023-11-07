
#include <cpp_pubsub/srv/modify_msg.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using StringMod = cpp_pubsub::srv::StringMod;

/**
 * @brief Function to process the request, i.e., to promt a msg that the string
 * is modified.
 *
 * @param request
 * @param response
 */
void add(const std::shared_ptr<StringMod::Request> request,
         std::shared_ptr<StringMod::Response> response) {
  response->c = request->a + " " + request->b + " have been added.";
}

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("modify_msg_server");

  rclcpp::Service<StringMod>::SharedPtr service =
      node->create_service<StringMod>("modify_msg", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modifying Msg");

  rclcpp::spin(node);
  rclcpp::shutdown();
}