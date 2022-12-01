/**
 * @file main.cpp
 * @author Harika Pendli (hpendli@umd.edu)
 * @brief Rostest to test the talker node.
 * @version 0.1
 * @date 2022-12-01
 * 
 */
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}