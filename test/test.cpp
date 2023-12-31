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

/**
 * @file test.cpp
 * @author Akashkumar Parmar (akasparm@umd.edu)
 * @brief
 * @version 0.1
 * @date 2023-11-12
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

/**
 * @brief Class for testing the publisher
 *
 */
class TaskPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr test_node_;
};

/**
 * @brief Construct a new test f object
 *
 */
TEST_F(TaskPublisher, test_num_publishers) {
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub =
      test_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto number_of_publishers = test_node_->count_publishers("chatter");

  // Check number of publishers
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}
