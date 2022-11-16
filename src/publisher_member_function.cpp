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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

#include "../include/beginner_tutorials/MinimalPublisher.hpp"
//#include <beginner_tutorials/srv/rename_string.hpp>

auto main_string = std::string("This is my main string");

MinimalPublisher::MinimalPublisher() 
  : Node("minimal_publisher"), count_(0) {

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer( 500ms, std::bind(&MinimalPublisher::timer_callback, this));

  
    auto serviceCallbackPtr = std::bind (&MinimalPublisher::change_base_string_srv, this, std::placeholders::_1, std::placeholders::_2);
    service_ = create_service <beginner_tutorials::srv::RenameString> ("update_string",serviceCallbackPtr);
}

void MinimalPublisher::timer_callback() {

  auto message = std_msgs::msg::String();
  message.data = main_string + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
  
}

void MinimalPublisher::change_base_string_srv(const std::shared_ptr<beginner_tutorials::srv::RenameString::Request> request,  // CHANGE
           std::shared_ptr<beginner_tutorials::srv::RenameString::Response> response)  // CHANGE
{
  response-> out = request->inp;                                      // CHANGE
  main_string = response -> out;
  RCLCPP_INFO(this->get_logger(), "Incoming request\na: '%s'" ,  // CHANGE
                request->inp.c_str());                                         // CHANGE
  RCLCPP_INFO(this->get_logger(), "sending back response: '%s'", response->out.c_str());
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
