/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// Workspace
#include <holistic_fusion_ros2/extrinsics/StaticTransformsTf.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ona_se {

class OnaStaticTransforms : public holistic_fusion::StaticTransformsTf {
 public:
  OnaStaticTransforms(const std::shared_ptr<rclcpp::Node>& nodePtr);

  // Setters
  void setLioOdometryFrame(const std::string& s) { lidarOdometryFrame_ = s; }
  void setGnssFrame(const std::string& s) { gnssFrame_ = s; }
  void setWheelOdometryBetweenFrame(const std::string& s) { wheelOdometryBetweenFrame_ = s; }
  void setWheelLinearVelocityLeftFrame(const std::string& s) { wheelLinearVelocityLeftFrame_ = s; }
  void setWheelLinearVelocityRightFrame(const std::string& s) { wheelLinearVelocityRightFrame_ = s; }
  void setOdometryFrame(const std::string& s) { OdometryFrame_ = s; }

  // Getters
  const std::string& getLioOdometryFrame() const { return lidarOdometryFrame_; }
  const std::string& getGnssFrame() const { return gnssFrame_; }
  const std::string& getWheelOdometryBetweenFrame() const { return wheelOdometryBetweenFrame_; }
  const std::string& getWheelLinearVelocityLeftFrame() const { return wheelLinearVelocityLeftFrame_; }
  const std::string& getWheelLinearVelocityRightFrame() const { return wheelLinearVelocityRightFrame_; }
  const std::string& getOdometryFrame() const { return OdometryFrame_; }

  // Set flags
  void setUseLioUnaryFlag(bool flag) { useLioUnaryFlag_ = flag; }
  void setUseGnssFlag(bool flag) { useGnssUnaryFlag_ = flag; }
  void setUseOdometryFlag(bool flag) { useOdometryFlag_ = flag; }
  void setUseWheelOdometryBetweenFlag(bool flag) { useWheelOdometryBetweenFlag_ = flag; }
  void setUseWheelLinearVelocitiesFlag(bool flag) { useWheelLinearVelocitiesFlag_ = flag; }

 private:
  bool findTransformations() override;

  // Frames
  std::string lidarOdometryFrame_;
  std::string gnssFrame_;
  std::string wheelOdometryBetweenFrame_;
  std::string wheelLinearVelocityLeftFrame_;
  std::string wheelLinearVelocityRightFrame_;
  std::string OdometryFrame_;

  // Odometry flags
  bool useLioUnaryFlag_ = false;
  bool useGnssUnaryFlag_ = false;
  bool useOdometryFlag_ = false;
  bool useWheelOdometryBetweenFlag_ = false;
  bool useWheelLinearVelocitiesFlag_ = false;
};

}  // namespace Ona_se
