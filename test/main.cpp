#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Main funtion to run all the tests
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}