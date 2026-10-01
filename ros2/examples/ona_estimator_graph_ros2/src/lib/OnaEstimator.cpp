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

// Workspace
#include "holistic_fusion/measurements/BinaryMeasurementXD.h"
#include "holistic_fusion/measurements/UnaryMeasurementXD.h"
#include "holistic_fusion_ros2/util/conversions.h"
#include "ona_estimator_graph_ros2/constants.h"

namespace ona_se {

OnaEstimator::OnaEstimator(const std::string& nodeName, const rclcpp::NodeOptions& options) : holistic_fusion::HolisticFusionRos2(nodeName, options) {
  REGULAR_COUT << GREEN_START << " OnaEstimator-Constructor called." << COLOR_END << std::endl;
}

void OnaEstimator::setup() {
  REGULAR_COUT << GREEN_START << " OnaEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  this->declare_parameter("sensor_params.useLioOdometry", false);
  this->declare_parameter("sensor_params.useGnss", false);
  this->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  this->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  this->declare_parameter("sensor_params.useOdometry", false);

  // Sensor parameters (int)
  this->declare_parameter("sensor_params.lioOdometryRate", 0.0);
  this->declare_parameter("sensor_params.gnssRate", 0.0);
  this->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0.0);
  this->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0.0);
  this->declare_parameter("sensor_params.OdometryRate", 0.0);

  // GNSS config
  this->declare_parameter("gnss.initYaw", 0.0);
  this->declare_parameter("gnss.useYawInitialGuessFromFile", false);
  this->declare_parameter("gnss.yawInitialGuessFromAlignment", false);
  this->declare_parameter("gnss.useGnssReference", false);
  this->declare_parameter("gnss.referenceLatitude", 41.38879);
  this->declare_parameter("gnss.referenceLongitude", 2.15899);
  this->declare_parameter("gnss.referenceAltitude", 12.0);
  this->declare_parameter("gnss.referenceHeading", 0.0);

  // Trajectory alignment config
  this->declare_parameter("trajectoryAlignment.minimumDistanceHeadingInit", 3.0);
  this->declare_parameter("trajectoryAlignment.noMovementDistance", 0.1);
  this->declare_parameter("trajectoryAlignment.noMovementTime", 1.0);


  // Alignment parameters (vector of double)
  this->declare_parameter("alignment_params.initialSe3AlignmentStdDev", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  this->declare_parameter("noise_params.lioPoseUnaryStdDev", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.odometryPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.gnssPositionOutlierThreshold", 1.0);

  // Extrinsic frames (string)
  this->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.gnssFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  this->declare_parameter("extrinsics.OdometryFrame", std::string(""));

  // Wheel Radius (double)
  this->declare_parameter("sensor_params.wheelRadius", 0.0);

  // Create OnaStaticTransforms
  staticTransformsPtr_ = std::make_shared<OnaStaticTransforms>(shared_from_this());

  OnaEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  OnaEstimator::initializePublishers();
  OnaEstimator::initializeSubscribers();
  OnaEstimator::initializeMessages();
  OnaEstimator::initializeServices();

  HolisticFusionRos2::setup(staticTransformsPtr_);

  // Transforms --> query until returns true
  bool foundTransforms = false;
  while (!foundTransforms) {
    foundTransforms = staticTransformsPtr_->findTransformations();
    // Sleep for 0.1 seconds to avoid busy waiting
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void OnaEstimator::initializePublishers() {
  pubMeasMapLioPath_ = this->create_publisher<nav_msgs::msg::Path>("/holistic_fusion/measLiDAR_path_map_imu", ROS_QUEUE_SIZE_ONA);
  pubMeasMapOdomPath_ = this->create_publisher<nav_msgs::msg::Path>("/holistic_fusion/measOdometry_path_world_imu", ROS_QUEUE_SIZE_ONA);
  pubMeasWorldGnssPath_ = this->create_publisher<nav_msgs::msg::Path>("/holistic_fusion/measGNSS_path_world_gnss", ROS_QUEUE_SIZE_ONA);
}

void OnaEstimator::initializeSubscribers() {

  if (useGnssUnaryFlag_) {
    subGnssUnary_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss_fix_topic", ROS_QUEUE_SIZE_ONA, std::bind(&OnaEstimator::gnssFixCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized GNSS Fix subscriber with topic: /gnss_fix_topic" << std::endl;
  }

  if (useLioUnaryFlag_) {
    subLioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE_ONA, std::bind(&OnaEstimator::lidarUnaryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE_ONA, std::bind(&OnaEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", ROS_QUEUE_SIZE_ONA, std::bind(&OnaEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useOdometryFlag_) {
    subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry_topic", ROS_QUEUE_SIZE_ONA, std::bind(&OnaEstimator::odometryBetweenCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Odometry (LIO/VIO) subscriber with topic: /odometry_topic" << std::endl;
  }
}

void OnaEstimator::initializeMessages() {
  measLio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measOdom_worldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measGnss_worldGnssPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
}

void OnaEstimator::initializeServices() {
  // Nothing for now
}

void OnaEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr) {
  const rclcpp::Time new_imu_timestamp{imuPtr->header.stamp};

  if (holistic_fusion::HolisticFusion::areRollAndPitchInited() && !holistic_fusion::HolisticFusion::areYawAndPositionInited() && !useLioUnaryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useOdometryFlag_) {
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;

    holistic_fusion::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(),
        imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9, 1.0, Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 1));

    holistic_fusion::HolisticFusion::initYawAndPosition(unary6DMeasurement);
    holistic_fusion::HolisticFusion::pretendFirstMeasurementReceived();
  }
  // Remove if norm is larger than 100
  const double angular_velocity_norm = std::sqrt(imuPtr->angular_velocity.x * imuPtr->angular_velocity.x +
                imuPtr->angular_velocity.y * imuPtr->angular_velocity.y +
                imuPtr->angular_velocity.z * imuPtr->angular_velocity.z);
  const double linear_acceleration_norm = std::sqrt(imuPtr->linear_acceleration.x * imuPtr->linear_acceleration.x +
                imuPtr->linear_acceleration.y * imuPtr->linear_acceleration.y +
                imuPtr->linear_acceleration.z * imuPtr->linear_acceleration.z);
  if (angular_velocity_norm > 10) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU angular velocity is larger than 10 rad/s, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (linear_acceleration_norm > 100.0) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU linear acceleration norm is larger than 100 m/s^2, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  // Check timestamps strictly increase
  if (new_imu_timestamp == last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was duplicated, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (new_imu_timestamp < last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was before last included IMU measurement "
        " at time" << last_imu_timestamp_.seconds() << ", skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  last_imu_timestamp_ = new_imu_timestamp;

  holistic_fusion::HolisticFusionRos2::imuCallback(imuPtr);
}

void OnaEstimator::lidarUnaryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {
  static int lidarOdometryCallbackCounter__ = -1;
  static double lastLidarOdometryTimeK_ = 0.0;

  // Timestamp
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;

  // Check whether the callback rate is not exceeded
  if (lidarOdometryCallbackCounter__ >= 0 && lidarOdometryTimeK - lastLidarOdometryTimeK_ < (1.0 / lioOdometryRate_)) {
    return;  // Skip this callback if the rate is exceeded
  } else {
    lastLidarOdometryTimeK_ = lidarOdometryTimeK;  // Update the last timestamp
  }

  // Update the callback counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk;
  holistic_fusion::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarOdometryTimeK);
  }
  
  const std::string& lioOdometryFrame = dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();

  holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      holistic_fusion::RobustNorm::Huber(3.0), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  } else if (!useGnssUnaryFlag_) { // Initializing if no GNSS
    this->initYawAndPosition(unary6DMeasurement);
  }

  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id  + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
               (lio_T_M_Lk * staticTransformsPtr_->rv_T_frame1_frame2(lioOdometryFrame, staticTransformsPtr_->getImuFrame()).matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  pubMeasMapLioPath_->publish(*measLio_mapImuPathPtr_);
}

void OnaEstimator::gnssFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnssMsgPtr)
{
  // Counter
  ++gnssCallbackCounter_;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssMsgPtr->position_covariance[0]), sqrt(gnssMsgPtr->position_covariance[4]),
                               sqrt(gnssMsgPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    REGULAR_COUT << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  // gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();  // Alias
  // fixedFrame = "east_north_up";

  // Initial world yaw initialization options
  // Case 1: Initialization
  if (!areYawAndPositionInited()) {
    // a: Default
    double initYaw_W_Base{0.0};  // Default is 0 yaw
    // b: From file
    if (gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      initYaw_W_Base = gnssHandlerPtr_->getGlobalYawDegFromFile() / 180.0 * M_PI;
    }
    // c: From alignment
    else if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addR3Position(W_t_W_Gnss, rclcpp::Time(gnssMsgPtr->header.stamp).seconds());
      // In radians
      Eigen::Isometry3d T_W_Base = Eigen::Isometry3d::Identity();
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base, T_W_Base))) {
        if (gnssCallbackCounter_ % 10 == 0) {
          REGULAR_COUT << YELLOW_START << "Trajectory alignment not ready. Waiting for more motion." << COLOR_END << std::endl;
        }
        return;
      }
      REGULAR_COUT << GREEN_START << "Trajectory Alignment Successful. Obtained Yaw Value of T_W_Base (deg): " << COLOR_END
                   << 180.0 * initYaw_W_Base / M_PI << std::endl;
    }

    // Actual Initialization
    if (this->initYawAndPositionInWorld(initYaw_W_Base, W_t_W_Gnss,
                                        dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                        dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << std::endl;

    } else {
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << std::endl;
    }
  } else {  // Case 2: Already initialized --> Unary factor
    const std::string& gnssFrameName = dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame();  // Alias
    // Measurement
    holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(),
        rclcpp::Time(gnssMsgPtr->header.stamp).seconds(), gnssPositionOutlierThreshold_, W_t_W_Gnss, estStdDevXYZ, fixedFrame,
        staticTransformsPtr_->getWorldFrame());
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  // Add _gmsf to the frame
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, fixedFrame, gnssMsgPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength_ * 4);
  /// Publish path
  pubMeasWorldGnssPath_->publish(*measGnss_worldGnssPathPtr_);
}

void OnaEstimator::wheelOdometryPoseCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdometryKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  ++wheelOdometryCallbackCounter_;

  Eigen::Isometry3d T_O_Bw_k;
  holistic_fusion::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.sec + wheelOdometryKPtr->header.stamp.nanosec * 1e-9;

  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  const std::string& wheelOdometryFrame = dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioUnaryFlag_) {
      holistic_fusion::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          holistic_fusion::RobustNorm::None(), wheelOdometryTimeK, 1.0, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      holistic_fusion::HolisticFusion::initYawAndPosition(unary6DMeasurement);
    }
  } else if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
    Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
    holistic_fusion::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Wheel_odometry_6D", int(wheelOdometryBetweenRate_ / 5), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
        holistic_fusion::RobustNorm::Tukey(1.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
    this->addBinaryPose3Measurement(delta6DMeasurement);

    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
  }
}

void OnaEstimator::wheelLinearVelocitiesCallback_(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& wheelsSpeedsPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = wheelsSpeedsPtr->data[0];
  const double leftWheelSpeedRps = wheelsSpeedsPtr->data[1];
  const double rightWheelSpeedRps = wheelsSpeedsPtr->data[2];
  const double leftWheelSpeedMs = leftWheelSpeedRps * wheelRadiusMeter_;
  const double rightWheelSpeedMs = rightWheelSpeedRps * wheelRadiusMeter_;

  const std::string& wheelLinearVelocityLeftFrame =
      dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityLeftFrame();
  const std::string& wheelLinearVelocityRightFrame =
      dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityRightFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioUnaryFlag_ && !useWheelOdometryBetweenFlag_) {
      holistic_fusion::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
          wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(), timeK, 1.0,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      holistic_fusion::HolisticFusion::initYawAndPosition(unary6DMeasurement);
    }
  } else {
    holistic_fusion::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    holistic_fusion::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void OnaEstimator::odometryBetweenCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& OdomPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++odomBetweenCallbackCounter_;

  // Convert
  Eigen::Isometry3d odom_T_M_Lk = Eigen::Isometry3d::Identity();
  holistic_fusion::odomMsgToEigen(*OdomPtr, odom_T_M_Lk.matrix());
  // Get the time
  double odomBetweenTimeK = rclcpp::Time(OdomPtr->header.stamp).seconds();

  // At start
  if (odomBetweenCallbackCounter_ == 0) {
    odom_T_M_Lkm1_ = odom_T_M_Lk;
    odomBetweenTimeKm1_ = odomBetweenTimeK;
  }

  // Add to trajectory aligner if needed.
  // if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
  //   trajectoryAlignmentHandler_->addSe3Position(odom_T_M_Lk.translation(), odomBetweenTimeK);
  // }

  // Frame Name
  const std::string& OdomFrameName = dynamic_cast<OnaStaticTransforms*>(staticTransformsPtr_.get())->getOdometryFrame();  // alias

  // State Machine
  if (odomBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_) {
      // Measurement
      holistic_fusion::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Odom_unary_6D", int(OdometryRate_), OdomFrameName, OdomFrameName + sensorFrameCorrectedNameId,
          holistic_fusion::RobustNorm::None(), odomBetweenTimeK, 1.0, odom_T_M_Lk, lioPoseUnaryNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << "Odometry (LIO/VIO) callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor
    // Compute Delta
    const Eigen::Isometry3d T_Lkm1_Lk = odom_T_M_Lkm1_.inverse() * odom_T_M_Lk;
    // Create measurement
    holistic_fusion::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Odom_between_6D", int(OdometryRate_), OdomFrameName, OdomFrameName + sensorFrameCorrectedNameId,
        holistic_fusion::RobustNorm::None(), odomBetweenTimeKm1_, odomBetweenTimeK, T_Lkm1_Lk, lioPoseUnaryNoise_);
    // Add to graph
    this->addBinaryPose3Measurement(delta6DMeasurement);
  }
  // Provide for next iteration
  odom_T_M_Lkm1_ = odom_T_M_Lk;
  odomBetweenTimeKm1_ = odomBetweenTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measOdom_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), OdomPtr->header.stamp, odom_T_M_Lk.translation(),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapOdomPath_->publish(*measOdom_worldImuPathPtr_);
}

}  // namespace ona_se
