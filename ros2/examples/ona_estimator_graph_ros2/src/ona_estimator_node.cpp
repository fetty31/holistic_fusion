/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// ROS 2
#include <rclcpp/rclcpp.hpp>

// Local packages
#include "ona_estimator_graph_ros2/OnaEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create Instance of OnaEstimator (which is itself a Node)
  auto OnaEstimator = std::make_shared<ona_se::OnaEstimator>("ona_estimator_node");

  // Call setup after construction (needs shared_from_this)
  OnaEstimator->setup();

  // Use Multi-Threaded Executor
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(OnaEstimator);
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}