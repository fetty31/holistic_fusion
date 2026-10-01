/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "ona_estimator_graph_ros2/OnaEstimator.h"

// Project
#include "ona_estimator_graph_ros2/OnaStaticTransforms.h"
#include "ona_estimator_graph_ros2/constants.h"

// HolisticFusion ROS2
#include "holistic_fusion_ros2/ros/read_ros_params.h"

namespace ona_se {

void OnaEstimator::readParams() {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("OnaEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLioUnaryFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useLioOdometry");
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->setUseLioUnaryFlag(useLioUnaryFlag_);
  useGnssUnaryFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useGnss");
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->setUseGnssFlag(useGnssUnaryFlag_);
  useWheelOdometryBetweenFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useWheelOdometryBetween");
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelOdometryBetweenFlag(useWheelOdometryBetweenFlag_);
  useWheelLinearVelocitiesFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useWheelLinearVelocities");
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelLinearVelocitiesFlag(useWheelLinearVelocitiesFlag_);
  useOdometryFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useOdometry");
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->setUseOdometryFlag(useOdometryFlag_);

  // Sensor Params
  lioOdometryRate_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.lioOdometryRate");
  gnssRate_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.gnssRate");
  wheelOdometryBetweenRate_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.wheelOdometryBetweenRate");
  wheelLinearVelocitiesRate_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.wheelLinearVelocitiesRate");
  OdometryRate_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.OdometryRate");

  // Gnss parameters ---------------------------------------------------
  if (useGnssUnaryFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<holistic_fusion::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(holistic_fusion::tryGetParam<bool>(this, "gnss.useYawInitialGuessFromFile"));
    gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(holistic_fusion::tryGetParam<bool>(this, "gnss.yawInitialGuessFromAlignment"));

    // Alignment options.
    if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      // Make sure no dual true
      gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
      trajectoryAlignmentHandler_ = std::make_shared<holistic_fusion::TrajectoryAlignmentHandler>();

      trajectoryAlignmentHandler_->setSe3Rate(lioOdometryRate_);
      trajectoryAlignmentHandler_->setR3Rate(gnssRate_);

      trajectoryAlignmentHandler_->setMinDistanceHeadingInit(
          holistic_fusion::tryGetParam<double>(this, "trajectoryAlignment.minimumDistanceHeadingInit"));
      trajectoryAlignmentHandler_->setNoMovementDistance(
          holistic_fusion::tryGetParam<double>(this, "trajectoryAlignment.noMovementDistance"));
      trajectoryAlignmentHandler_->setNoMovementTime(holistic_fusion::tryGetParam<double>(this, "trajectoryAlignment.noMovementTime"));

    } else if (!gnssHandlerPtr_->getUseYawInitialGuessFromAlignment() && gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      gnssHandlerPtr_->setGlobalYawDegFromFile(holistic_fusion::tryGetParam<double>(this, "gnss.initYaw"));
    }

    // GNSS Reference
    gnssHandlerPtr_->setUseGnssReferenceFlag(holistic_fusion::tryGetParam<bool>(this, "gnss.useGnssReference"));

    if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
      REGULAR_COUT << GREEN_START << " Using GNSS reference from parameters." << COLOR_END << std::endl;
      gnssHandlerPtr_->setGnssReferenceLatitude(holistic_fusion::tryGetParam<double>(this, "gnss.referenceLatitude"));
      gnssHandlerPtr_->setGnssReferenceLongitude(holistic_fusion::tryGetParam<double>(this, "gnss.referenceLongitude"));
      gnssHandlerPtr_->setGnssReferenceAltitude(holistic_fusion::tryGetParam<double>(this, "gnss.referenceAltitude"));
      gnssHandlerPtr_->setGnssReferenceHeading(holistic_fusion::tryGetParam<double>(this, "gnss.referenceHeading"));
    } else {
      REGULAR_COUT << GREEN_START << " Will wait for GNSS measurements to initialize reference coordinates." << COLOR_END << std::endl;
    }

    // GNSS Outlier Threshold
    gnssPositionOutlierThreshold_ = holistic_fusion::tryGetParam<double>(this, "noise_params.gnssPositionOutlierThreshold");
  }  // End GNSS Unary

  // Alignment Parameters
  const auto initialSe3AlignmentStdDev =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "alignment_params.initialSe3AlignmentStdDev");
  initialSe3AlignmentNoise_ << initialSe3AlignmentStdDev[0], initialSe3AlignmentStdDev[1], initialSe3AlignmentStdDev[2],
      initialSe3AlignmentStdDev[3], initialSe3AlignmentStdDev[4], initialSe3AlignmentStdDev[5];
  const auto lioSe3AlignmentRandomWalk =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "alignment_params.lioSe3AlignmentRandomWalk");
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
            holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseUnaryStdDev");  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  /// Between
  const auto wheelPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.wheelPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// Linear Velocities
  const auto wheelLinearVelocitiesNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.wheelLinearVelocitiesNoiseDensity");  // left,right
  wheelLinearVelocitiesNoise_ << wheelLinearVelocitiesNoise[0], wheelLinearVelocitiesNoise[1], wheelLinearVelocitiesNoise[2];
  /// Odometry (LIO/VIO)
  const auto odometryPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.odometryPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  odometryPoseBetweenNoise_ << odometryPoseBetweenNoise[0], odometryPoseBetweenNoise[1], odometryPoseBetweenNoise[2], odometryPoseBetweenNoise[3],
      odometryPoseBetweenNoise[4], odometryPoseBetweenNoise[5];

  // Set frames
  /// LiDAR odometry frame
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.lidarOdometryFrame"));
  /// LiDAR odometry frame
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.gnssFrame"));
  /// Wheel Odometry frame
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryBetweenFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelOdometryBetweenFrame"));
  /// Whel Linear Velocities frames
  /// Left
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityLeftFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityLeftFrame"));
  /// Right
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityRightFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityRightFrame"));

  /// VIO/LIO Odometry frame
  dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())
      ->setOdometryFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.OdometryFrame"));

  // Wheel Radius
  wheelRadiusMeter_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.wheelRadius");
}

}  // namespace ona_se
