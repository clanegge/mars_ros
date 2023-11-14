// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mars_wrapper_dualpose_position.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_ros/ExtCoreState.h>
#include <mars_ros/ExtCoreStateLite.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>

using namespace mars;

MarsWrapperDPosePosition::MarsWrapperDPosePosition(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperDPosePosition::configCallback, this, _1, _2))
  , p_wi_init_(0, 0, 0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
  , m_sett_(nh)
{
  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperDPosePosition::initServiceCallback, this);

  std::cout << "Setup framework components" << std::endl;

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_initial_covariance(m_sett_.core_init_cov_p_, m_sett_.core_init_cov_v_,
                                                  m_sett_.core_init_cov_q_, m_sett_.core_init_cov_bw_,
                                                  m_sett_.core_init_cov_ba_);

  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(m_sett_.buffer_size_);

  core_logic_.verbose_ = m_sett_.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett_.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett_.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(
      Eigen::Vector3d(m_sett_.g_rate_noise_, m_sett_.g_rate_noise_, m_sett_.g_rate_noise_),
      Eigen::Vector3d(m_sett_.g_bias_noise_, m_sett_.g_bias_noise_, m_sett_.g_bias_noise_),
      Eigen::Vector3d(m_sett_.a_noise_, m_sett_.a_noise_, m_sett_.a_noise_),
      Eigen::Vector3d(m_sett_.a_bias_noise_, m_sett_.a_bias_noise_, m_sett_.a_bias_noise_));

  // Sensors
  {
    pose1_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose1", core_states_sptr_);
    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett_.pose1_pos_meas_noise_, m_sett_.pose1_rot_meas_noise_;
    pose1_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);
    pose1_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.pose1_use_dyn_meas_noise_;

    mars::PoseSensorData pose_calibration;
    pose_calibration.state_.p_ip_ = m_sett_.pose1_cal_p_ip_;
    pose_calibration.state_.q_ip_ = m_sett_.pose1_cal_q_ip_;

    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << m_sett_.pose1_state_init_cov_;
    pose_calibration.sensor_cov_ = pose_cov;

    pose1_sensor_sptr_->set_initial_calib(std::make_shared<PoseSensorData>(pose_calibration));
    // TODO(CHB) is set here for now, but will be managed by core logic in later versions
    pose1_sensor_sptr_->const_ref_to_nav_ = true;
  }
  {
    pose2_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose2", core_states_sptr_);
    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett_.pose2_pos_meas_noise_, m_sett_.pose2_rot_meas_noise_;
    pose2_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);
    pose2_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.pose2_use_dyn_meas_noise_;

    mars::PoseSensorData pose_calibration;
    pose_calibration.state_.p_ip_ = m_sett_.pose2_cal_p_ip_;
    pose_calibration.state_.q_ip_ = m_sett_.pose2_cal_q_ip_;

    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << m_sett_.pose2_state_init_cov_;
    pose_calibration.sensor_cov_ = pose_cov;

    pose2_sensor_sptr_->set_initial_calib(std::make_shared<PoseSensorData>(pose_calibration));
    pose2_sensor_sptr_->const_ref_to_nav_ = false;
  }
  {
    position1_sensor_sptr_ = std::make_shared<mars::PositionSensorClass>("Position1", core_states_sptr_);
    Eigen::Matrix<double, 3, 1> position_meas_std;
    position_meas_std << m_sett_.position1_pos_meas_noise_;
    position1_sensor_sptr_->R_ = position_meas_std.cwiseProduct(position_meas_std);
    position1_sensor_sptr_->use_dynamic_meas_noise_ = m_sett_.position1_use_dyn_meas_noise_;

    PositionSensorData position_calibration;
    position_calibration.state_.p_ip_ = m_sett_.position1_cal_p_ip_;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                 "!!!!1"
              << position_calibration.state_.p_ip_ << std::endl;

    Eigen::Matrix<double, 3, 3> position_cov;
    position_cov.setZero();
    position_cov.diagonal() << m_sett_.position1_state_init_cov_;
    position_calibration.sensor_cov_ = position_cov;

    position1_sensor_sptr_->set_initial_calib(std::make_shared<PositionSensorData>(position_calibration));
    position1_sensor_sptr_->const_ref_to_nav_ = false;
  }

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", m_sett_.sub_imu_cb_buffer_size_, &MarsWrapperDPosePosition::ImuMeasurementCallback, this);
  sub_pose_measurement_ = nh.subscribe("pose_in", m_sett_.sub_sensor_cb_buffer_size_,
                                       &MarsWrapperDPosePosition::PoseMeasurementCallback, this);
  sub_pose_with_cov_measurement_ = nh.subscribe("pose_with_cov_in", m_sett_.sub_sensor_cb_buffer_size_,
                                                &MarsWrapperDPosePosition::PoseWithCovMeasurementCallback, this);
  sub_odom_measurement_ = nh.subscribe("odom_in", m_sett_.sub_sensor_cb_buffer_size_,
                                       &MarsWrapperDPosePosition::OdomMeasurementCallback, this);
  sub_odom_measurement2_ = nh.subscribe("odom2_in", m_sett_.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDPosePosition::OdomMeasurement2Callback, this);
  sub_point_measurement_ = nh.subscribe("point_in", m_sett_.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDPosePosition::PointMeasurementCallback, this);
  sub_transform_measurement_ = nh.subscribe("transform_in", m_sett_.sub_sensor_cb_buffer_size_,
                                            &MarsWrapperDPosePosition::TransformMeasurementCallback, this);

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", m_sett_.pub_cb_buffer_size_);
  pub_ext_core_state_lite_ =
      nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", m_sett_.pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", m_sett_.pub_cb_buffer_size_);
  pub_core_odom_state_ = nh.advertise<nav_msgs::Odometry>("core_odom_state_out", m_sett_.pub_cb_buffer_size_);
  pub_pose1_state_ = nh.advertise<geometry_msgs::PoseStamped>("pose_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_pose2_state_ = nh.advertise<geometry_msgs::PoseStamped>("pose2_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_position1_state_ =
      nh.advertise<geometry_msgs::PoseStamped>("position_cal_state_out", m_sett_.pub_cb_buffer_size_);

  if (m_sett_.pub_path_)
  {
    pub_core_path_ = nh.advertise<nav_msgs::Path>("core_states_path", m_sett_.pub_cb_buffer_size_);
  }
}

bool MarsWrapperDPosePosition::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  pose1_sensor_sptr_->is_initialized_ = false;
  pose2_sensor_sptr_->is_initialized_ = false;
  position1_sensor_sptr_->is_initialized_ = false;
  have_pose1_ = false;
  have_pose2_ = false;
  have_position1_ = false;

  return true;
}

bool MarsWrapperDPosePosition::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                                                   std_srvs::SetBool::Response& response)
{
  init();
  ROS_INFO_STREAM("Initialized filter trough ROS Service");

  response.success = true;
  return true;
}

void MarsWrapperDPosePosition::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
{
  // Config parameter overwrite
  m_sett_.publish_on_propagation_ = config.pub_on_prop;
  core_logic_.verbose_ = config.verbose;
  m_sett_.verbose_output_ = config.verbose;
  m_sett_.use_ros_time_now_ = config.use_ros_time_now;

  if (config.initialize)
  {
    init();
    ROS_INFO_STREAM("Initialized filter trough Reconfigure GUI");
  }

  config.initialize = false;
}

void MarsWrapperDPosePosition::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
{
  // Map the measutement to the mars type
  Time timestamp;

  if (m_sett_.use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<IMUMeasurementType>(MarsMsgConv::ImuMsgToImuMeas(*meas)));

  // Call process measurement
  const bool valid_update = core_logic_.ProcessMeasurement(imu_sensor_sptr_, timestamp, data);

  // Initialize the first time at which the propagation sensor occures
  if (!core_logic_.core_is_initialized_ && have_pose1_)
  {
    core_logic_.Initialize(p_wi_init_, q_wi_init_);
  }

  if (m_sett_.publish_on_propagation_ && valid_update)
  {
    this->RunCoreStatePublisher();
  }
}

void MarsWrapperDPosePosition::PoseMeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDPosePosition::PoseWithCovMeasurementCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDPosePosition::TransformMeasurementCallback(const geometry_msgs::TransformStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::TransformMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDPosePosition::OdomMeasurementCallback(const nav_msgs::OdometryConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDPosePosition::OdomMeasurement2Callback(const nav_msgs::OdometryConstPtr& meas)
{
  if (have_pose1_)  // Wait until we have received first pose
  {
    Time timestamp(meas->header.stamp.toSec());
    PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
    PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
  }
}

void MarsWrapperDPosePosition::PointMeasurementCallback(const geometry_msgs::PointStampedConstPtr& meas)
{
  if (have_pose1_)  // Wait until we have received first pose
  {
    // Map the measurement to the mars sensor type
    Time timestamp(meas->header.stamp.toSec());

    PositionMeasurementType position_meas = MarsMsgConv::PointMsgToPositionMeas(*meas);
    PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
  }
}

void MarsWrapperDPosePosition::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;
  const bool valid_state = core_logic_.buffer_.get_latest_state(&latest_state);

  if (!valid_state)
  {
    return;
  }

  mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

  if (m_sett_.pub_cov_)
  {
    mars::CoreStateMatrix cov = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->cov_;
    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsgCov(latest_state.timestamp_.get_seconds(), latest_core_state, cov));
  }
  else
  {
    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  }

  pub_ext_core_state_lite_.publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_pose_state_.publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_odom_state_.publish(
      MarsMsgConv::ExtCoreStateToOdomMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  if (m_sett_.pub_path_)
  {
    pub_core_path_.publish(
        path_generator_.ExtCoreStateToPathMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  }
}

void MarsWrapperDPosePosition::PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
                                                     const PoseMeasurementType& pose_meas, const Time& timestamp)
{
  Time timestamp_corr;

  if (m_sett_.use_ros_time_now_)
  {
    timestamp_corr = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp_corr = timestamp;
  }

  PoseMeasurementType rotated_pose_meas = pose_meas;

  // TMP feedback init pose (with sensor no1)
  if (sensor_sptr->name_ == "Pose1")
  {
    //    p_wi_init_ = pose_meas.position_;
    //    q_wi_init_ = pose_meas.orientation_;
    have_pose1_ = true;
  }
  else if (sensor_sptr->name_ == "Pose2")
  {
    /// \todo TODO(scm): this should probably be a pose_sensor conversion class!
    if (have_pose2_)
    {
      // measurement is in 'global' frame of pose 2 (gp2) to pose sensor
      // calibration is from vision (pose 1 global frame, and Mars global frame, i.e. w) to gp2
      // hence
      // p_wp = p_w_gp2 (here IP) + R_w_gp2 (here IP) * meas_gp2_p
      // q_wp = q_w_gp2 (here IP) * meas_gp2_p
      rotated_pose_meas.position_ =
          pose12_calibration_.state_.p_ip_ + pose12_calibration_.state_.q_ip_.toRotationMatrix() * pose_meas.position_;
      rotated_pose_meas.orientation_ = pose12_calibration_.state_.q_ip_ * pose_meas.orientation_;
    }
    else if (core_logic_.core_is_initialized_)
    {
      // get current core state
      mars::BufferEntryType latest_state;
      core_logic_.buffer_.get_latest_state(&latest_state);
      mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

      // save sensor 1 to sensor 2 calibration!
      /// \note NOTE(scm): variable names are wrong here, reusage of existing variables with their naming conventions,
      /// however it should be noted that _ip rather stands for _v_gp2, i.e. from the vision frame to the 'global' frame
      /// of pose sensor 2.
      Eigen::Matrix3d R_ip = latest_core_state.q_wi_.toRotationMatrix() * m_sett_.pose2_cal_q_ip_ *
                             pose_meas.orientation_.toRotationMatrix().transpose();
      pose12_calibration_.state_.q_ip_ = Eigen::Quaterniond(R_ip);
      pose12_calibration_.state_.p_ip_ = latest_core_state.p_wi_ - R_ip * pose_meas.position_;
      Eigen::Matrix<double, 6, 6> pose_cov;

      ROS_INFO_STREAM("Pose2 initial calib: \n\tpos: " << pose12_calibration_.state_.p_ip_.transpose()
                                                       << "\n\tori: " << pose12_calibration_.state_.q_ip_.w() << " "
                                                       << pose12_calibration_.state_.q_ip_.vec().transpose());

      have_pose2_ = true;
    }
    else
      return;
  }

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<PoseMeasurementType>(rotated_pose_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
  {
    return;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  // Publish the latest sensor state
  mars::BufferEntryType latest_result;
  const bool valid_state = core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_result);

  if (!valid_state)
  {
    return;
  }

  mars::PoseSensorStateType pose_sensor_state = sensor_sptr.get()->get_state(latest_result.data_.sensor_);
  if (sensor_sptr->name_ == "Pose1")
  {
    pub_pose1_state_.publish(
        MarsMsgConv::PoseStateToPoseMsg(latest_result.timestamp_.get_seconds(), pose_sensor_state));
  }
  else if (sensor_sptr->name_ == "Pose2")
  {
    pub_pose2_state_.publish(
        MarsMsgConv::PoseStateToPoseMsg(latest_result.timestamp_.get_seconds(), pose_sensor_state));
  }
  else
  {
    std::cout << "Could not publish pose state.. sensor not found: " << sensor_sptr->name_ << std::endl;
  }
}

void MarsWrapperDPosePosition::PositionMeasurementUpdate(std::shared_ptr<mars::PositionSensorClass> sensor_sptr,
                                                         const mars::PositionMeasurementType& position_meas,
                                                         const mars::Time& timestamp)
{
  Time timestamp_corr;

  if (m_sett_.use_ros_time_now_)
  {
    timestamp_corr = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp_corr = timestamp;
  }

  // TMP feedback init position
  //  p_wi_init_ = position_meas.position_;

  PositionMeasurementType rotated_position_meas = position_meas;
  if (have_position1_)
  {
    rotated_position_meas.position_ =
        pose1position_calibration_.state_.p_ip_ +
        pose1position_calibration_.state_.q_ip_.toRotationMatrix() * position_meas.position_;
  }
  else if (core_logic_.core_is_initialized_)
  {
    // get current core state
    mars::BufferEntryType latest_state;
    core_logic_.buffer_.get_latest_state(&latest_state);
    mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;
    Eigen::Matrix3d R_ip = latest_core_state.q_wi_.toRotationMatrix() * m_sett_.position1_cal_q_ip_;
    pose1position_calibration_.state_.q_ip_ = Eigen::Quaterniond(R_ip);
    pose1position_calibration_.state_.p_ip_ = latest_core_state.p_wi_ - R_ip * position_meas.position_;
    have_position1_ = true;
  }
  else
  {
    return;
  }

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<PositionMeasurementType>(rotated_position_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
  {
    return;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  // Publish the latest sensor state
  mars::BufferEntryType latest_result;
  const bool valid_state = core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_result);

  if (!valid_state)
  {
    return;
  }

  mars::PositionSensorStateType position_sensor_state = sensor_sptr.get()->get_state(latest_result.data_.sensor_);

  pub_position1_state_.publish(
      MarsMsgConv::PositionStateToPoseWithCovMsg(latest_result.timestamp_.get_seconds(), position_sensor_state, "imu"));
}
