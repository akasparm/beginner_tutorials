#include <cpp_pubsub/srv/modify_msg.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using ModifyMsg = cpp_pubsub::srv::ModifyMsg;

/**
 * @brief Function to process the request, i.e., to promt a msg that the string
 * is modified.
 *
 * @param request
 * @param response
 */
void add(const std::shared_ptr<ModifyMsg::Request> request,
         std::shared_ptr<ModifyMsg::Response> response) {
  response->c = "String has been modified to "+ request->a + "-" + request->b;
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

  rclcpp::Service<ModifyMsg>::SharedPtr service =
      node->create_service<ModifyMsg>("modify_msg", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Started.. Modifying Msg");

  rclcpp::spin(node);
  rclcpp::shutdown();
}