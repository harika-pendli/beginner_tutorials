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

#include <signal.h>

#include "../include/beginner_tutorials/MinimalSubscriber.hpp"

/**
 * @brief Construct a new Minimal Subscriber:: Minimal Subscriber object
 *
 */
MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

  if (this->count_publishers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "No publishers on this topic to listen");
  }
}

void MinimalSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) const {
  RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->data);
}

/**
 * @brief returns an error stream message when forced to shutdown using ctrl+c
 *
 * @param signum
 */
void node_forcestop(int signum) {
  if (signum == 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Force stopped! Bye!");
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "Call to end node worked ");
  }
}

int main(int argc, char* argv[]) {
  signal(SIGINT, node_forcestop);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
