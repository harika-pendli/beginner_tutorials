/**
 * @file publisherTest.cpp
 * @author Harika Pendli (hpendli@umd.edu)
 * @brief Source file to implement a simple ROS test
 * @version 0.1
 * @date 2022-12-01
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "../include/beginner_tutorials/MinimalPublisher.hpp"


class TestTalker : public testing::Test {
 public:
  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestTalker, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("topic", 10.0);

  auto num_pub = node_->count_publishers("topic");
  EXPECT_EQ(1, num_pub);
}