/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <iterator>
#include <algorithm>

#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <msckf_vio/msckf_vio.h>
#include <msckf_vio/math_utils.hpp>
#include <msckf_vio/utils.h>

#include "msckf_vio/logging.h"
#include "msckf_vio/tic_toc.h"

using namespace std;
using namespace Eigen;

#define WITH_SPQR 0
#define WITH_HHQR 0
#define WITH_GIVENS_QR 1

#define DEBUG_IMG_J 0

namespace msckf_vio{

// Static member variables in IMUState class.
StateIDType IMUState::next_id = 0;
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Vector3d IMUState::gravity = Vector3d(0, 0, -GRAVITY_ACCELERATION);
Isometry3d IMUState::T_imu_body = Isometry3d::Identity();

// Static member variables in CAMState class.
Isometry3d CAMState::T_cam0_cam1 = Isometry3d::Identity();

// Static member variables in Feature class.
FeatureIDType Feature::next_id = 0;
double Feature::observation_noise = 0.01;
Feature::OptimizationConfig Feature::optimization_config;

map<int, double> MsckfVio::chi_squared_test_table;

MsckfVio::MsckfVio(ros::NodeHandle& pnh): is_gravity_set(false), is_first_img(true), nh(pnh) { return; }

bool MsckfVio::loadParameters() {
    // Frame id
    nh.param<string>("fixed_frame_id", fixed_frame_id, "world");
    nh.param<string>("child_frame_id", child_frame_id, "robot");
    nh.param<bool>("publish_tf", publish_tf, true);
    nh.param<double>("frame_rate", frame_rate, 40.0);
    nh.param<double>("position_std_threshold", position_std_threshold, 8.0);

    nh.param<double>("rotation_threshold", rotation_threshold, 0.2618);
    nh.param<double>("translation_threshold", translation_threshold, 0.4);
    nh.param<double>("tracking_rate_threshold", tracking_rate_threshold, 0.5);

    // Feature optimization parameters
    nh.param<double>("feature/config/translation_threshold", Feature::optimization_config.translation_threshold, 0.2);

    // Noise related parameters
    nh.param<double>("noise/gyro", IMUState::gyro_noise, 0.001);
    nh.param<double>("noise/acc", IMUState::acc_noise, 0.01);
    nh.param<double>("noise/gyro_bias", IMUState::gyro_bias_noise, 0.001);
    nh.param<double>("noise/acc_bias", IMUState::acc_bias_noise, 0.01);
    nh.param<double>("noise/feature", Feature::observation_noise, 0.01);

    // Use variance instead of standard deviation.
    IMUState::gyro_noise *= IMUState::gyro_noise;
    IMUState::acc_noise *= IMUState::acc_noise;
    IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
    IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
    Feature::observation_noise *= Feature::observation_noise;

    // Set the initial IMU state.
    // The intial orientation and position will be set to the origin
    // implicitly. But the initial velocity and bias can be set by parameters.
    // TODO: is it reasonable to set the initial bias to 0?
    nh.param<double>("initial_state/velocity/x", state_server.imu_state.velocity(0), 0.0);
    nh.param<double>("initial_state/velocity/y", state_server.imu_state.velocity(1), 0.0);
    nh.param<double>("initial_state/velocity/z", state_server.imu_state.velocity(2), 0.0);

    // The initial covariance of orientation and position can be
    // set to 0. But for velocity, bias and extrinsic parameters,
    // there should be nontrivial uncertainty.
    double gyro_bias_cov, acc_bias_cov, velocity_cov;
    nh.param<double>("initial_covariance/velocity", velocity_cov, 0.25);
    nh.param<double>("initial_covariance/gyro_bias", gyro_bias_cov, 1e-4);
    nh.param<double>("initial_covariance/acc_bias", acc_bias_cov, 1e-2);

    double extrinsic_rotation_cov, extrinsic_translation_cov;
    nh.param<double>("initial_covariance/extrinsic_rotation_cov", extrinsic_rotation_cov, 3.0462e-4);
    nh.param<double>("initial_covariance/extrinsic_translation_cov", extrinsic_translation_cov, 1e-4);

    state_server.state_cov = MatrixXd::Zero(21, 21);
    for (int i = 3; i < 6; ++i)
        state_server.state_cov(i, i) = gyro_bias_cov;
    for (int i = 6; i < 9; ++i)
        state_server.state_cov(i, i) = velocity_cov;
    for (int i = 9; i < 12; ++i)
        state_server.state_cov(i, i) = acc_bias_cov;
    for (int i = 15; i < 18; ++i)
        state_server.state_cov(i, i) = extrinsic_rotation_cov;
    for (int i = 18; i < 21; ++i)
        state_server.state_cov(i, i) = extrinsic_translation_cov;

    // Transformation offsets between the frames involved.
    Isometry3d T_imu_cam0 = utils::getTransformEigen(nh, "cam0/T_cam_imu"); // Tci
    Isometry3d T_cam0_imu = T_imu_cam0.inverse(); // Tic

    state_server.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose(); // Rci
    state_server.imu_state.t_cam0_imu = T_cam0_imu.translation(); // tic
    CAMState::T_cam0_cam1 = utils::getTransformEigen(nh, "cam1/T_cn_cnm1");
    IMUState::T_imu_body = utils::getTransformEigen(nh, "T_imu_body").inverse();

    // Maximum number of camera states to be stored
    nh.param<int>("max_cam_state_size", max_cam_state_size, 30);

    ROS_INFO("MsckfVio begin ===========================================");

    ROS_INFO("fixed frame id: %s", fixed_frame_id.c_str());
    ROS_INFO("child frame id: %s", child_frame_id.c_str());
    ROS_INFO("publish tf: %d", publish_tf);
    ROS_INFO("frame rate: %f", frame_rate);
    ROS_INFO("position std threshold: %f", position_std_threshold);
    ROS_INFO("Keyframe rotation threshold: %f", rotation_threshold);
    ROS_INFO("Keyframe translation threshold: %f", translation_threshold);
    ROS_INFO("Keyframe tracking rate threshold: %f", tracking_rate_threshold);
    ROS_INFO("gyro noise: %.10f", IMUState::gyro_noise);
    ROS_INFO("gyro bias noise: %.10f", IMUState::gyro_bias_noise);
    ROS_INFO("acc noise: %.10f", IMUState::acc_noise);
    ROS_INFO("acc bias noise: %.10f", IMUState::acc_bias_noise);
    ROS_INFO("observation noise: %.10f", Feature::observation_noise);
    ROS_INFO("initial velocity: %f, %f, %f",
             state_server.imu_state.velocity(0),
             state_server.imu_state.velocity(1),
             state_server.imu_state.velocity(2));
    ROS_INFO("initial gyro bias cov: %f", gyro_bias_cov);
    ROS_INFO("initial acc bias cov: %f", acc_bias_cov);
    ROS_INFO("initial velocity cov: %f", velocity_cov);
    ROS_INFO("initial extrinsic rotation cov: %f", extrinsic_rotation_cov);
    ROS_INFO("initial extrinsic translation cov: %f", extrinsic_translation_cov);

    cout << T_imu_cam0.linear() << endl;
    cout << T_imu_cam0.translation().transpose() << endl;

    ROS_INFO("max camera state #: %d", max_cam_state_size);

    std::cout << "T_imu_cam0:\n" << T_imu_cam0.matrix() << std::endl;
    std::cout << "T_cam0_cam1:\n" << CAMState::T_cam0_cam1.matrix() << std::endl;
    std::cout << "T_imu_body:\n" << IMUState::T_imu_body.matrix() << std::endl;

    ROS_INFO("MsckfVio end ===========================================");
    return true;
}

bool MsckfVio::createRosIO() {
    path_pub = nh.advertise<nav_msgs::Path>("path", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    cam_odom_pub = nh.advertise<nav_msgs::Odometry>("odom_cam", 10);
    feature_pub = nh.advertise<sensor_msgs::PointCloud2>("feature_point_cloud", 10);

    reset_srv = nh.advertiseService("reset", &MsckfVio::resetCallback, this);

    imu_sub = nh.subscribe("imu", 100, &MsckfVio::imuCallback, this);
    feature_sub = nh.subscribe("features", 40, &MsckfVio::featureCallback, this);

    mocap_odom_sub = nh.subscribe("mocap_odom", 10, &MsckfVio::mocapOdomCallback, this);
    mocap_odom_pub = nh.advertise<nav_msgs::Odometry>("gt_odom", 1);

#if WITH_LC
    pub_poseimu = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 2);
    pub_keyframe_pose = nh.advertise<nav_msgs::Odometry>("/ov_msckf/keyframe_pose", 1000);
    pub_keyframe_point = nh.advertise<sensor_msgs::PointCloud>("/ov_msckf/keyframe_feats", 1000);    
    pub_keyframe_extrinsic = nh.advertise<nav_msgs::Odometry>("/ov_msckf/keyframe_extrinsic", 1000);
    pub_keyframe_intrinsics = nh.advertise<sensor_msgs::CameraInfo>("/ov_msckf/keyframe_intrinsics", 1000);
#endif    

    return true;
}

bool MsckfVio::initialize() {
    if (!loadParameters())
        return false;
    ROS_INFO("Finish loading ROS parameters...");

    // Initialize state server
    state_server.continuous_noise_cov = Matrix<double, 12, 12>::Zero();
    state_server.continuous_noise_cov.block<3, 3>(0, 0) = Matrix3d::Identity() * IMUState::gyro_noise;
    state_server.continuous_noise_cov.block<3, 3>(3, 3) = Matrix3d::Identity() * IMUState::gyro_bias_noise;
    state_server.continuous_noise_cov.block<3, 3>(6, 6) = Matrix3d::Identity() * IMUState::acc_noise;
    state_server.continuous_noise_cov.block<3, 3>(9, 9) = Matrix3d::Identity() * IMUState::acc_bias_noise;

    // Initialize the chi squared test table with confidence level 0.95.
    for (int i = 1; i < 100; ++i) chi_squared_test_table[i] = utils::chi_square_table_p95[i-1];

    if (!createRosIO())
        return false;
    ROS_INFO("Finish creating ROS IO...");

    pose_outfile_.open("msckf_pose_out.tum");

    return true;
}

void MsckfVio::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    // IMU msgs are pushed backed into a buffer instead of being processed immediately.
    // The IMU msgs are processed when the next image is available, in which way, we can easily handle the transfer delay.
    imu_msg_buffer.push_back(*msg);

    if (!is_gravity_set) {
        if (imu_msg_buffer.size() < 200)
            return;
        //if (imu_msg_buffer.size() < 10) return;
        initializeGravityAndBias();
        is_gravity_set = true;
    }

    return;
}

// QXC：这个初始化方法不对加计零偏做补偿，可能会出问题 ???
void MsckfVio::initializeGravityAndBias() {
    // Initialize gravity and gyro bias.
    Vector3d sum_angular_vel = Vector3d::Zero();
    Vector3d sum_linear_acc  = Vector3d::Zero();

    for (const auto &imu_msg : imu_msg_buffer) {
        Vector3d angular_vel = Vector3d::Zero();
        Vector3d linear_acc  = Vector3d::Zero();

        tf::vectorMsgToEigen(imu_msg.angular_velocity, angular_vel);
        tf::vectorMsgToEigen(imu_msg.linear_acceleration, linear_acc);

        sum_angular_vel += angular_vel;
        sum_linear_acc  += linear_acc;
    }

    // 平均角速度作为陀螺仪的bias
    state_server.imu_state.gyro_bias = sum_angular_vel / imu_msg_buffer.size();
    // IMU系下重力向量 This is the gravity in the IMU frame
    Vector3d gravity_imu = sum_linear_acc / imu_msg_buffer.size();

    // Initialize the initial orientation, so that the estimation is consistent with the inertial frame.
    double gravity_norm = gravity_imu.norm(); // 平均加速度的模值作为重力加速度模值g
    IMUState::gravity = Vector3d(0.0, 0.0, -gravity_norm); // World系下重力向量（天向）

    // 计算初始时刻 World系重力向量(0,0,g) 和 IMU系下重力向量 之间的姿态(旋转四元数)
    Quaterniond q0_i_w = Quaterniond::FromTwoVectors(gravity_imu, -IMUState::gravity);
    state_server.imu_state.orientation = rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

    return;
}

bool MsckfVio::resetCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res) {

    ROS_WARN("Start resetting msckf vio...");
    // Temporarily shutdown the subscribers to prevent the
    // state from updating.
    feature_sub.shutdown();
    imu_sub.shutdown();

    // Reset the IMU state.
    IMUState &imu_state = state_server.imu_state;
    imu_state.time = 0.0;
    imu_state.orientation = Vector4d(0.0, 0.0, 0.0, 1.0);
    imu_state.position = Vector3d::Zero();
    imu_state.velocity = Vector3d::Zero();
    imu_state.gyro_bias = Vector3d::Zero();
    imu_state.acc_bias = Vector3d::Zero();
    imu_state.orientation_null = Vector4d(0.0, 0.0, 0.0, 1.0);
    imu_state.position_null = Vector3d::Zero();
    imu_state.velocity_null = Vector3d::Zero();

    // Remove all existing camera states.
    state_server.cam_states.clear();

    // Reset the state covariance.
    double gyro_bias_cov, acc_bias_cov, velocity_cov;
    nh.param<double>("initial_covariance/velocity", velocity_cov, 0.25);
    nh.param<double>("initial_covariance/gyro_bias", gyro_bias_cov, 1e-4);
    nh.param<double>("initial_covariance/acc_bias", acc_bias_cov, 1e-2);

    double extrinsic_rotation_cov, extrinsic_translation_cov;
    nh.param<double>("initial_covariance/extrinsic_rotation_cov", extrinsic_rotation_cov, 3.0462e-4);
    nh.param<double>("initial_covariance/extrinsic_translation_cov", extrinsic_translation_cov, 1e-4);

    state_server.state_cov = MatrixXd::Zero(21, 21);
    for (int i = 3; i < 6; ++i)
        state_server.state_cov(i, i) = gyro_bias_cov;
    for (int i = 6; i < 9; ++i)
        state_server.state_cov(i, i) = velocity_cov;
    for (int i = 9; i < 12; ++i)
        state_server.state_cov(i, i) = acc_bias_cov;
    for (int i = 15; i < 18; ++i)
        state_server.state_cov(i, i) = extrinsic_rotation_cov;
    for (int i = 18; i < 21; ++i)
        state_server.state_cov(i, i) = extrinsic_translation_cov;

    // Clear all exsiting features in the map.
    map_server.clear();

    // Clear the IMU msg buffer.
    imu_msg_buffer.clear();

    // Reset the starting flags.
    is_gravity_set = false;
    is_first_img = true;

    // Restart the subscribers.
    imu_sub = nh.subscribe("imu", 100, &MsckfVio::imuCallback, this);
    feature_sub = nh.subscribe("features", 40, &MsckfVio::featureCallback, this);

    // TODO: When can the reset fail?
    res.success = true;
    ROS_WARN("Resetting msckf vio completed...");
    return true;
}

void MsckfVio::featureCallback(const CameraMeasurementConstPtr& msg) {
    // Return if the gravity vector has not been set.
    if (!is_gravity_set)
        return;

    // Start the system if the first image is received.
    // The frame where the first image is received will be the origin.
    if (is_first_img) {
        is_first_img = false;
        state_server.imu_state.time = msg->header.stamp.toSec();
    }

    static int critical_time_cntr = 0;
    double processing_start_time = ros::Time::now().toSec();

    // Propogate the IMU state. that are received before the image msg.
    ros::Time start_time = ros::Time::now();
    batchImuProcessing(msg->header.stamp.toSec());
    double imu_processing_time = (ros::Time::now() - start_time).toSec();

    // Augment the state vector.
    start_time = ros::Time::now();
    stateAugmentation(msg->header.stamp.toSec());
    double state_augmentation_time = (ros::Time::now() - start_time).toSec();

    // Add new observations for existing features or new features in the map server.
    start_time = ros::Time::now();
    addFeatureObservations(msg);
    double add_observations_time = (ros::Time::now() - start_time).toSec();

    // Perform measurement update if necessary.
    start_time = ros::Time::now();
    removeLostFeatures();
    double remove_lost_features_time = (ros::Time::now() - start_time).toSec();

    start_time = ros::Time::now();
    pruneCamStateBuffer();
    double prune_cam_states_time = (ros::Time::now() - start_time).toSec();

#if WITH_LC
    std::vector<FeatureLcPtr> featuresLC;
    for(const auto &feat : featuresLCDB) featuresLC.emplace_back(feat.second);
    featuresLCDB.clear();
    update_keyframe_historical_information(featuresLC);
    publish_keyframe_information();
#endif    

    // Publish the odometry.
    start_time = ros::Time::now();
    publish(msg->header.stamp);
    double publish_time = (ros::Time::now() - start_time).toSec();

    // Reset the system if necessary.
    onlineReset();

    double processing_end_time = ros::Time::now().toSec();
    double processing_time = processing_end_time - processing_start_time;
    if (processing_time > 1.0 / frame_rate) {
        ++critical_time_cntr;
        printf("\033[1;31mTotal processing time %f/%d...\033[0m\n", processing_time, critical_time_cntr);
        printf("IMU processing time: %f/%f\n", imu_processing_time, imu_processing_time/processing_time);
        printf("State augmentation time: %f/%f\n", state_augmentation_time, state_augmentation_time/processing_time);
        printf("Add observations time: %f/%f\n", add_observations_time, add_observations_time/processing_time);
        printf("Remove lost features time: %f/%f\n", remove_lost_features_time, remove_lost_features_time / processing_time);
        printf("Remove camera states time: %f/%f\n", prune_cam_states_time, prune_cam_states_time / processing_time);
        printf("Publish time: %f/%f\n", publish_time, publish_time/processing_time);
    }

    return;
}

void MsckfVio::mocapOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  static bool first_mocap_odom_msg = true;

  // If this is the first mocap odometry messsage, set the initial frame.
  if (first_mocap_odom_msg) {
    Quaterniond orientation;
    Vector3d translation;
    tf::pointMsgToEigen(msg->pose.pose.position, translation);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);
    //tf::vectorMsgToEigen(msg->transform.translation, translation);
    //tf::quaternionMsgToEigen(msg->transform.rotation, orientation);
    mocap_initial_frame.linear() = orientation.toRotationMatrix();
    mocap_initial_frame.translation() = translation;
    first_mocap_odom_msg = false;
  }

  // Transform the ground truth.
  Quaterniond orientation;
  Vector3d translation;
  //tf::vectorMsgToEigen(msg->transform.translation, translation);
  //tf::quaternionMsgToEigen(msg->transform.rotation, orientation);
  tf::pointMsgToEigen(msg->pose.pose.position, translation);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);

  Eigen::Isometry3d T_b_v_gt;
  T_b_v_gt.linear()      = orientation.toRotationMatrix();
  T_b_v_gt.translation() = translation;
  Eigen::Isometry3d T_b_w_gt = mocap_initial_frame.inverse() * T_b_v_gt;

  //Eigen::Vector3d body_velocity_gt;
  //tf::vectorMsgToEigen(msg->twist.twist.linear, body_velocity_gt);
  //body_velocity_gt = mocap_initial_frame.linear().transpose() * body_velocity_gt;

  // Ground truth tf.
  if (publish_tf) {
    tf::Transform T_b_w_gt_tf;
    tf::transformEigenToTF(T_b_w_gt, T_b_w_gt_tf);
    tf_pub.sendTransform(tf::StampedTransform(T_b_w_gt_tf, msg->header.stamp, fixed_frame_id, child_frame_id+"_mocap"));
  }

  // Ground truth odometry.
  nav_msgs::Odometry mocap_odom_msg;
  mocap_odom_msg.header.stamp = msg->header.stamp;
  mocap_odom_msg.header.frame_id = fixed_frame_id;
  mocap_odom_msg.child_frame_id = child_frame_id+"_mocap";

  tf::poseEigenToMsg(T_b_w_gt, mocap_odom_msg.pose.pose);
  //tf::vectorEigenToMsg(body_velocity_gt, mocap_odom_msg.twist.twist.linear);

  mocap_odom_pub.publish(mocap_odom_msg);
  return;
}

void MsckfVio::batchImuProcessing(const double& time_bound) {
    // Counter how many IMU msgs in the buffer are used.
    int used_imu_msg_cntr = 0;

    for (const auto &imu_msg : imu_msg_buffer) {
        double imu_time = imu_msg.header.stamp.toSec();
        if (imu_time < state_server.imu_state.time) {
            ++used_imu_msg_cntr;
            continue;
        }
        if (imu_time > time_bound)
            break;

        // Convert the msgs.
        Vector3d m_gyro, m_acc;
        tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
        tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);

        // Execute process model.
        processModel(imu_time, m_gyro, m_acc);
        ++used_imu_msg_cntr;
    }

    // Set the state ID for the new IMU state.
    state_server.imu_state.id = IMUState::next_id++;

    // Remove all used IMU msgs.
    imu_msg_buffer.erase(imu_msg_buffer.begin(), imu_msg_buffer.begin() + used_imu_msg_cntr);

    return;
}

void MsckfVio::processModel(const double& time, const Vector3d& m_gyro, const Vector3d& m_acc) {
    // Remove the bias from the measured gyro and acceleration
    IMUState &imu_state = state_server.imu_state;
    Vector3d gyro = m_gyro - imu_state.gyro_bias;
    Vector3d acc  = m_acc  - imu_state.acc_bias;
    double dtime  = time - imu_state.time;

    /// IMU 状态转移矩阵F 和 噪声协方差矩阵G
    // Compute discrete transition and noise covariance matrix
    Matrix<double, 21, 21> F = Matrix<double, 21, 21>::Zero();
    Matrix<double, 21, 12> G = Matrix<double, 21, 12>::Zero();

    F.block<3, 3>(0, 0) = -skewSymmetric(gyro);
    F.block<3, 3>(0, 3) = -Matrix3d::Identity();
    F.block<3, 3>(6, 0) = -quaternionToRotation(imu_state.orientation).transpose() * skewSymmetric(acc);
    F.block<3, 3>(6, 9) = -quaternionToRotation(imu_state.orientation).transpose();
    F.block<3, 3>(12, 6) = Matrix3d::Identity();

    G.block<3, 3>(0, 0) = -Matrix3d::Identity();
    G.block<3, 3>(3, 3) =  Matrix3d::Identity();
    G.block<3, 3>(6, 6) = -quaternionToRotation(imu_state.orientation).transpose();
    G.block<3, 3>(9, 9) =  Matrix3d::Identity();

    /// 系统状态转移矩阵Phi
    // Approximate matrix exponential to the 3rd order,
    // which can be considered to be accurate enough assuming dtime is within 0.01s.
    Matrix<double, 21, 21> Fdt = F * dtime;
    Matrix<double, 21, 21> Fdt_square = Fdt * Fdt;
    Matrix<double, 21, 21> Fdt_cube = Fdt_square * Fdt;
    Matrix<double, 21, 21> Phi = Matrix<double, 21, 21>::Identity() + Fdt + 0.5 * Fdt_square + (1.0 / 6.0) * Fdt_cube;

    /// 状态向量预测
    // Propogate the state using 4th order Runge-Kutta
    predictNewState(dtime, gyro, acc);

    // TODO[cg]: why
    // Modify the transition matrix to make the observability matrix have proper null space
    Matrix3d R_kk_1 = quaternionToRotation(imu_state.orientation_null);
    Phi.block<3, 3>(0, 0) = quaternionToRotation(imu_state.orientation) * R_kk_1.transpose();

    Vector3d u = R_kk_1 * IMUState::gravity;
    RowVector3d s = (u.transpose() * u).inverse() * u.transpose();

    Matrix3d A1 = Phi.block<3, 3>(6, 0);
    Vector3d w1 = skewSymmetric(imu_state.velocity_null - imu_state.velocity) * IMUState::gravity;
    Phi.block<3, 3>(6, 0) = A1 - (A1 * u - w1) * s;

    Matrix3d A2 = Phi.block<3, 3>(12, 0);
    Vector3d w2 = skewSymmetric(dtime * imu_state.velocity_null + imu_state.position_null - imu_state.position) * IMUState::gravity;
    Phi.block<3, 3>(12, 0) = A2 - (A2 * u - w2) * s;

    /// 系统噪声协方差矩阵Q
    // Propogate the state covariance matrix.
    Matrix<double, 21, 21> Q = Phi * G * state_server.continuous_noise_cov * G.transpose() * Phi.transpose() * dtime;

    /// 系统状态协方差矩阵P预测
    state_server.state_cov.block<21, 21>(0, 0) = Phi * state_server.state_cov.block<21, 21>(0, 0) * Phi.transpose() + Q;
    if (state_server.cam_states.size() > 0) {
        state_server.state_cov.block(0, 21, 21, state_server.state_cov.cols() - 21) =
                Phi * state_server.state_cov.block(0, 21, 21, state_server.state_cov.cols() - 21);
        state_server.state_cov.block(21, 0, state_server.state_cov.rows() - 21, 21) =
                state_server.state_cov.block(21, 0, state_server.state_cov.rows() - 21, 21) * Phi.transpose();
    }

    // Fix the covariance to be symmetric
    MatrixXd state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;

    // Update the state correspondes to null space.
    imu_state.orientation_null = imu_state.orientation;
    imu_state.position_null = imu_state.position;
    imu_state.velocity_null = imu_state.velocity;

    // Update the state info
    state_server.imu_state.time = time;
    return;
}

void MsckfVio::predictNewState(const double& dt, const Vector3d& gyro, const Vector3d& acc) {
    // TODO: Will performing the forward integration using the inverse of the quaternion give better accuracy?
    Matrix4d Omega = Matrix4d::Zero();
    Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
    Omega.block<3, 1>(0, 3) =  gyro;
    Omega.block<1, 3>(3, 0) = -gyro;

    Vector4d &q = state_server.imu_state.orientation;
    Vector3d &v = state_server.imu_state.velocity;
    Vector3d &p = state_server.imu_state.position;

    /// 姿态预测（0阶四元数积分）
    // Some pre-calculation
    Vector4d dq_dt, dq_dt2;
    double gyro_norm = gyro.norm();
    if (gyro_norm > 1e-5) {
        dq_dt  = (cos(gyro_norm * dt * 0.5)  * Matrix4d::Identity() + 1 / gyro_norm * sin(gyro_norm * dt * 0.5)  * Omega) * q;
        dq_dt2 = (cos(gyro_norm * dt * 0.25) * Matrix4d::Identity() + 1 / gyro_norm * sin(gyro_norm * dt * 0.25) * Omega) * q;
    } else {
        dq_dt  = (Matrix4d::Identity() + 0.5  * dt * Omega) * cos(gyro_norm * dt * 0.5)  * q; // TODO[cg]: why cos(gyro_norm * dt * 0.5)
        dq_dt2 = (Matrix4d::Identity() + 0.25 * dt * Omega) * cos(gyro_norm * dt * 0.25) * q;
    }
    Matrix3d dR_dt_transpose  = quaternionToRotation(dq_dt).transpose();
    Matrix3d dR_dt2_transpose = quaternionToRotation(dq_dt2).transpose();

    /// 速度和位置预测（4阶Runge-Kutta积分）
    // k1 = f(tn, yn)
    Vector3d k1_v_dot = quaternionToRotation(q).transpose() * acc + IMUState::gravity;
    Vector3d k1_p_dot = v;

    // k2 = f(tn+dt/2, yn+k1*dt/2)
    Vector3d k1_v = v + k1_v_dot * dt / 2;
    Vector3d k2_v_dot = dR_dt2_transpose * acc + IMUState::gravity;
    Vector3d k2_p_dot = k1_v;

    // k3 = f(tn+dt/2, yn+k2*dt/2)
    Vector3d k2_v = v + k2_v_dot * dt / 2;
    Vector3d k3_v_dot = dR_dt2_transpose * acc + IMUState::gravity;
    Vector3d k3_p_dot = k2_v;

    // k4 = f(tn+dt, yn+k3*dt)
    Vector3d k3_v = v + k3_v_dot * dt;
    Vector3d k4_v_dot = dR_dt_transpose * acc + IMUState::gravity;
    Vector3d k4_p_dot = k3_v;

    // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
    q = dq_dt;
    quaternionNormalize(q);
    v = v + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    p = p + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

    return;
}

void MsckfVio::stateAugmentation(const double& time) {
    const Matrix3d &R_i_c = state_server.imu_state.R_imu_cam0; // Rci
    const Vector3d &t_c_i = state_server.imu_state.t_cam0_imu; // tic

    /// 相机状态向量扩增
    // Add a new camera state to the state server.
    Matrix3d R_w_i = quaternionToRotation(state_server.imu_state.orientation); // Riw
    Matrix3d R_w_c = R_i_c * R_w_i; // Rcw
    Vector3d t_c_w = state_server.imu_state.position + R_w_i.transpose() * t_c_i; // twc

    state_server.cam_states[state_server.imu_state.id] = CAMState(state_server.imu_state.id);
    CAMState &cam_state = state_server.cam_states[state_server.imu_state.id];

    cam_state.time = time;
    cam_state.orientation = rotationToQuaternion(R_w_c);
    cam_state.position    = t_c_w;

    cam_state.orientation_null = cam_state.orientation;
    cam_state.position_null    = cam_state.position;

    /// 状态协方差矩阵扩增
    // Update the covariance matrix of the state.
    // To simplify computation, the matrix J below is the nontrivial block
    // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation".
    Matrix<double, 6, 21> J = Matrix<double, 6, 21>::Zero();
    J.block<3, 3>(0, 0)  = R_i_c;
    J.block<3, 3>(0, 15) = Matrix3d::Identity();
    J.block<3, 3>(3, 0)  = skewSymmetric(R_w_i.transpose() * t_c_i);
//    J.block<3, 3>(3, 0) = -R_w_i.transpose()*skewSymmetric(t_c_i);
    J.block<3, 3>(3, 12) = Matrix3d::Identity();
    // J.block<3, 3>(3, 18) = Matrix3d::Identity();
   J.block<3, 3>(3, 18) = R_w_i.transpose();

    // Resize the state covariance matrix.
    size_t old_rows = state_server.state_cov.rows();
    size_t old_cols = state_server.state_cov.cols();
    state_server.state_cov.conservativeResize(old_rows + 6, old_cols + 6);

    // Rename some matrix blocks for convenience.
    const Matrix<double, 21, 21> &P11 = state_server.state_cov.block<21, 21>(0, 0);
    const MatrixXd &P12 = state_server.state_cov.block(0, 21, 21, old_cols - 21);

    // Fill in the augmented state covariance.
    state_server.state_cov.block(old_rows, 0, 6, old_cols) << J * P11, J * P12;
    state_server.state_cov.block(0, old_cols, old_rows, 6) =
            state_server.state_cov.block(old_rows, 0, 6, old_cols).transpose();
    state_server.state_cov.block<6, 6>(old_rows, old_cols) = J * P11 * J.transpose();

    // Fix the covariance to be symmetric
    MatrixXd state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;

    return;
}

void MsckfVio::addFeatureObservations(const CameraMeasurementConstPtr& msg) {
    StateIDType state_id = state_server.imu_state.id;
    int curr_feature_num = map_server.size();
    int tracked_feature_num = 0;

    // imu_state.id <--> feature.id
    // Add new observations for existing features or new features in the map server.
    for (const auto &feature : msg->features) {
        if (map_server.find(feature.id) == map_server.end()) {
            // This is a new feature.
            map_server[feature.id] = Feature(feature.id);
            map_server[feature.id].observations[state_id] = Vector4d(feature.u0, feature.v0, feature.u1, feature.v1);
#if WITH_LC            
            map_server[feature.id].observations_uvs[state_id] = Vector4d(feature.pu0, feature.pv0, feature.pu1, feature.pv1);
            map_server[feature.id].timestamp[state_id] = msg->header.stamp.toSec();
#endif            
        } else {
            // This is an old feature.
            map_server[feature.id].observations[state_id] = Vector4d(feature.u0, feature.v0, feature.u1, feature.v1);
#if WITH_LC            
            map_server[feature.id].observations_uvs[state_id] = Vector4d(feature.pu0, feature.pv0, feature.pu1, feature.pv1);
            map_server[feature.id].timestamp[state_id] = msg->header.stamp.toSec();
#endif            
            ++tracked_feature_num;
        }
    }

    tracking_rate = static_cast<double>(tracked_feature_num) / static_cast<double>(curr_feature_num);

    return;
}

void MsckfVio::measurementJacobian(
    const StateIDType& cam_state_id, const FeatureIDType& feature_id,
    Matrix<double, 4, 6>& H_x, Matrix<double, 4, 3>& H_f, Vector4d& r) {

    // Prepare all the required data.
    const CAMState &cam_state = state_server.cam_states[cam_state_id];
    const Feature &feature = map_server[feature_id];

    // Cam0 pose.
    Matrix3d R_w_c0 = quaternionToRotation(cam_state.orientation);
    const Vector3d &t_c0_w = cam_state.position;

    // Cam1 pose.
    Matrix3d R_c0_c1 = CAMState::T_cam0_cam1.linear();
    Matrix3d R_w_c1  = CAMState::T_cam0_cam1.linear() * R_w_c0;
    Vector3d t_c1_w  = t_c0_w - R_w_c1.transpose() * CAMState::T_cam0_cam1.translation();

    // 3d feature position in the world frame. And its observation with the stereo cameras.
    const Vector3d &p_w = feature.position;
    const Vector4d &z = feature.observations.find(cam_state_id)->second;

    // Convert the feature position from the world frame to the cam0 and cam1 frame.
    Vector3d p_c0 = R_w_c0 * (p_w - t_c0_w);
    Vector3d p_c1 = R_w_c1 * (p_w - t_c1_w);

    // Compute the Jacobians.
    Matrix<double, 4, 3> dz_dpc0 = Matrix<double, 4, 3>::Zero();
    dz_dpc0(0, 0) = 1 / p_c0(2);
    dz_dpc0(1, 1) = 1 / p_c0(2);
    dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

    Matrix<double, 4, 3> dz_dpc1 = Matrix<double, 4, 3>::Zero();
    dz_dpc1(2, 0) = 1 / p_c1(2);
    dz_dpc1(3, 1) = 1 / p_c1(2);
    dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2) * p_c1(2));
    dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2) * p_c1(2));

    Matrix<double, 3, 6> dpc0_dxc = Matrix<double, 3, 6>::Zero();
    dpc0_dxc.leftCols(3) = skewSymmetric(p_c0);
    dpc0_dxc.rightCols(3) = -R_w_c0;

    Matrix<double, 3, 6> dpc1_dxc = Matrix<double, 3, 6>::Zero();
    dpc1_dxc.leftCols(3) = R_c0_c1 * skewSymmetric(p_c0);
    dpc1_dxc.rightCols(3) = -R_w_c1;

    Matrix3d dpc0_dpg = R_w_c0;
    Matrix3d dpc1_dpg = R_w_c1;

    H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
    H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;

    // TODO[cg]: why
    // Modifty the measurement Jacobian to ensure observability constrain. Ref: OC-VINS
    Matrix<double, 4, 6> A = H_x;
    Matrix<double, 6, 1> u = Matrix<double, 6, 1>::Zero();
    u.block<3, 1>(0, 0) = quaternionToRotation(cam_state.orientation_null) * IMUState::gravity;
    u.block<3, 1>(3, 0) = skewSymmetric(p_w - cam_state.position_null) * IMUState::gravity;
    H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
    H_f = -H_x.block<4, 3>(0, 3);

    // Compute the residual.
    r = z - Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2), p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));

    return;
}

void MsckfVio::featureJacobian(
    const FeatureIDType& feature_id, const std::vector<StateIDType>& cam_state_ids,
    MatrixXd& H_x, VectorXd& r) {

    const auto &feature = map_server[feature_id];

    // Check how many camera states in the provided camera id camera has actually seen this feature.
    vector<StateIDType> valid_cam_state_ids(0);
    for (const auto &cam_id : cam_state_ids) {
        if (feature.observations.find(cam_id) == feature.observations.end())
            continue;

        valid_cam_state_ids.push_back(cam_id);
    }

    int jacobian_row_size = 0;
    jacobian_row_size = 4 * valid_cam_state_ids.size();

    MatrixXd H_xj = MatrixXd::Zero(jacobian_row_size, 21 + state_server.cam_states.size() * 6);
    MatrixXd H_fj = MatrixXd::Zero(jacobian_row_size, 3);
    VectorXd r_j = VectorXd::Zero(jacobian_row_size);

    int stack_cntr = 0;
    for (const auto &cam_id : valid_cam_state_ids) {
        Matrix<double, 4, 6> H_xi = Matrix<double, 4, 6>::Zero();
        Matrix<double, 4, 3> H_fi = Matrix<double, 4, 3>::Zero();
        Vector4d r_i = Vector4d::Zero();
        measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

        auto cam_state_iter = state_server.cam_states.find(cam_id);
        int cam_state_cntr = std::distance(state_server.cam_states.begin(), cam_state_iter);

        // Stack the Jacobians.
        H_xj.block<4, 6>(stack_cntr, 21 + 6 * cam_state_cntr) = H_xi;
        H_fj.block<4, 3>(stack_cntr, 0) = H_fi;
        r_j.segment<4>(stack_cntr) = r_i;
        stack_cntr += 4;
    }

    // Project the residual and Jacobians onto the nullspace of H_fj
#if WITH_GIVENS_QR
    nullspace_project_inplace(H_fj, H_xj, r_j);
    H_x = H_xj;
    r = r_j;
#elif 1
    JacobiSVD<MatrixXd> svd_helper(H_fj, ComputeFullU | ComputeThinV);
    MatrixXd A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);
    H_x = A.transpose() * H_xj;
    r   = A.transpose() * r_j;
#else
    Eigen::HouseholderQR<Eigen::MatrixXd> qr_helper(H_fj);
    Eigen::MatrixXd Q = qr_helper.householderQ();
    MatrixXd A = Q.rightCols(jacobian_row_size - 3);
    H_x = A.transpose() * H_xj;
    r   = A.transpose() * r_j;
#endif    

    return;
}

void MsckfVio::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

    // Apply the left nullspace of H_f to all variables
    // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
    // See page 252, Algorithm 5.2.4 for how these two loops work
    // They use "matlab" index notation, thus we need to subtract 1 from all index
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_f.cols(); ++n) {
        for (int m = (int) H_f.rows() - 1; m > n; m--) {
            // Givens matrix G
            tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
            // Multiply G to the corresponding lines (m-1,m) in each matrix
            // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
            //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
            (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        }
    }

    // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
    // NOTE: need to eigen3 eval here since this experiences aliasing!
    //H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
    H_x = H_x.block(H_f.cols(),0,H_x.rows()-H_f.cols(),H_x.cols()).eval();
    res = res.block(H_f.cols(),0,res.rows()-H_f.cols(),res.cols()).eval();

    // Sanity check
    assert(H_x.rows()==res.rows());
}

void MsckfVio::measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {
    // Return if H_x is a fat matrix (there is no need to compress in this case)
    if(H_x.rows() <= H_x.cols())
        return;

    // Do measurement compression through givens rotations
    // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
    // See page 252, Algorithm 5.2.4 for how these two loops work
    // They use "matlab" index notation, thus we need to subtract 1 from all index
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n=0; n<H_x.cols(); n++) {
        for (int m=(int)H_x.rows()-1; m>n; m--) {
            // Givens matrix G
            tempHo_GR.makeGivens(H_x(m-1,n), H_x(m,n));
            // Multiply G to the corresponding lines (m-1,m) in each matrix
            // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
            //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
            (H_x.block(m-1,n,2,H_x.cols()-n)).applyOnTheLeft(0,1,tempHo_GR.adjoint());
            (res.block(m-1,0,2,1)).applyOnTheLeft(0,1,tempHo_GR.adjoint());
        }
    }

    // If H is a fat matrix, then use the rows
    // Else it should be same size as our state
    int r = std::min(H_x.rows(),H_x.cols());

    // Construct the smaller jacobian and residual after measurement compression
    assert(r<=H_x.rows());
    H_x.conservativeResize(r, H_x.cols());
    res.conservativeResize(r, res.cols());
}

void MsckfVio::measurementUpdate(const MatrixXd& H, const VectorXd& r) {
    if (H.rows() == 0 || r.rows() == 0)
        return;

    // Decompose the final Jacobian matrix to reduce computational complexity as in Equation (28), (29).
    MatrixXd H_thin;
    VectorXd r_thin;

    // HouseholderQR 处理时间是 SPQR 的 6-7 倍
    // HouseholderQR 处理时间是 FullPivHouseholderQR 的 1.5-2.5 倍
    // SPQR 处理时间是 Givens QR 的 4 倍 以上
    if (H.rows() > H.cols()) {
#if WITH_SPQR
        TicToc t_spqr;
        SparseMatrix<double> H_sparse = H.sparseView(); // Convert H to a sparse matrix.
        SPQR<SparseMatrix<double> > spqr_helper;
        spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
        spqr_helper.compute(H_sparse);

        MatrixXd H_temp;
        VectorXd r_temp;
        (spqr_helper.matrixQ().transpose() * H).evalTo(H_temp);
        (spqr_helper.matrixQ().transpose() * r).evalTo(r_temp);

        H_thin = H_temp.topRows(21 + state_server.cam_states.size() * 6);
        r_thin = r_temp.head(21 + state_server.cam_states.size() * 6);
        LOGI("[cggos %s] t_spqr: %f ms\n", __FUNCTION__, t_spqr.toc());
#endif

#if WITH_HHQR
        TicToc t_hhqr;
        Eigen::HouseholderQR<MatrixXd> qr_helper(H);
        MatrixXd Q = qr_helper.householderQ();
        // Eigen::FullPivHouseholderQR<Eigen::MatrixXd> fphqr_helper = H.fullPivHouseholderQr();
        // Eigen::MatrixXd Q = fphqr_helper.matrixQ();
        MatrixXd Q1 = Q.leftCols(21+state_server.cam_states.size()*6);
        H_thin = Q1.transpose() * H;
        r_thin = Q1.transpose() * r;
        LOGI("[cggos %s] t_hhqr: %f ms\n", __FUNCTION__, t_hhqr.toc());
#endif

#if WITH_GIVENS_QR
        TicToc t_givens;
        H_thin = H;
        r_thin = r;
        measurement_compress_inplace(H_thin, r_thin);
        // LOGI("[cggos %s] t_givens: %f ms\n", __FUNCTION__, t_givens.toc());
#endif
    } else {
        H_thin = H;
        r_thin = r;
    }

    // Compute the Kalman gain.
    const MatrixXd &P = state_server.state_cov;

    MatrixXd HP  = H_thin * P;
    MatrixXd HPH = HP * H_thin.transpose();

    MatrixXd K;
    MatrixXd R = Feature::observation_noise * MatrixXd::Identity(H_thin.rows(), H_thin.rows());
    MatrixXd S(R.rows(), R.rows());
#if 1
    TicToc t_K01;
    S = HPH + R;
    //MatrixXd K_transpose = S.fullPivHouseholderQr().solve(H_thin*P);
    MatrixXd K_transpose = S.llt().solve(HP);
    K = K_transpose.transpose();
    // LOGI("[cggos %s] t_K01: %f ms\n", __FUNCTION__, t_K01.toc());
#else
    TicToc t_K02;
    S.triangularView<Eigen::Upper>() = HPH;
    S.triangularView<Eigen::Upper>() += R;
    Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
    S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);
    K = HP.transpose() * Sinv.selfadjointView<Eigen::Upper>();
    LOGI("[cggos %s] t_K02: %f ms\n", __FUNCTION__, t_K02.toc());
#endif    

    // Compute the error of the state.
    VectorXd delta_x = K * r_thin;

    // Update the IMU state.
    const VectorXd &delta_x_imu = delta_x.head<21>();

    if (//delta_x_imu.segment<3>(0).norm() > 0.15 ||
        //delta_x_imu.segment<3>(3).norm() > 0.15 ||
            delta_x_imu.segment<3>(6).norm() > 0.5 ||
            //delta_x_imu.segment<3>(9).norm() > 0.5 ||
            delta_x_imu.segment<3>(12).norm() > 1.0) {
        printf("delta velocity: %f\n", delta_x_imu.segment<3>(6).norm());
        printf("delta position: %f\n", delta_x_imu.segment<3>(12).norm());
        ROS_WARN("Update change is too large.");
        //return;
    }

    const Vector4d dq_imu = smallAngleQuaternion(delta_x_imu.head<3>());
    state_server.imu_state.orientation = quaternionMultiplication(dq_imu, state_server.imu_state.orientation);
    state_server.imu_state.gyro_bias += delta_x_imu.segment<3>(3);
    state_server.imu_state.velocity  += delta_x_imu.segment<3>(6);
    state_server.imu_state.acc_bias  += delta_x_imu.segment<3>(9);
    state_server.imu_state.position  += delta_x_imu.segment<3>(12);

    const Vector4d dq_extrinsic = smallAngleQuaternion(delta_x_imu.segment<3>(15));
    state_server.imu_state.R_imu_cam0 = quaternionToRotation(dq_extrinsic) * state_server.imu_state.R_imu_cam0;
    state_server.imu_state.t_cam0_imu += delta_x_imu.segment<3>(18);

    // Update the camera states.
    auto cam_state_iter = state_server.cam_states.begin();
    for (int i = 0; i < state_server.cam_states.size(); ++i, ++cam_state_iter) {
        const VectorXd &delta_x_cam = delta_x.segment<6>(21 + i * 6);
        const Vector4d dq_cam = smallAngleQuaternion(delta_x_cam.head<3>());
        cam_state_iter->second.orientation = quaternionMultiplication(dq_cam, cam_state_iter->second.orientation);
        cam_state_iter->second.position += delta_x_cam.tail<3>();
    }

    // Update state covariance.
    MatrixXd KHP = K * HP;
#if 0
    //state_server.state_cov = I_KH*state_server.state_cov*I_KH.transpose() + K*K.transpose()*Feature::observation_noise;
    state_server.state_cov -=  KHP;
    MatrixXd state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;
#else
    state_server.state_cov.triangularView<Eigen::Upper>() -= KHP;
    state_server.state_cov = state_server.state_cov.selfadjointView<Eigen::Upper>();
#endif
    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    Eigen::VectorXd diags = state_server.state_cov.diagonal();
    bool found_neg = false;
    for(int i=0; i<diags.rows(); i++) {
        if(diags(i) < 0.0) {
            printf("[cggos %s] ERROR: diagonal at %d is %.2f\n", __FUNCTION__, i, diags(i));
            found_neg = true;
        }
    }
    assert(!found_neg);    

#if DEBUG_IMG_J
    if(H.rows() > 10 && H.cols() > 10) {
        cv::Mat Jhx(H.rows(), H.cols(), CV_8UC1);
        for(int h=0; h<Jhx.rows; h++) for(int w=0; w<Jhx.cols; w++) {
            double val = H(h, w);
            Jhx.at<uchar>(h, w) = std::abs(val)>DBL_MIN ? 255 : 0;
        }
        cv::Mat Jhthin(H_thin.rows(), H_thin.cols(), CV_8UC1);
        for(int h=0; h<Jhthin.rows; h++) for(int w=0; w<Jhthin.cols; w++) {
            double val = H_thin(h, w);
            Jhthin.at<uchar>(h, w) = std::abs(val)>DBL_MIN ? 255 : 0;
        }
        cv::Mat mat_hx_hthin;
        cv::vconcat(Jhx, Jhthin, mat_hx_hthin);
        cv::imshow("mat_hx_hthin", mat_hx_hthin);
        cv::waitKey(5);        
    }
#endif

    return;
}

bool MsckfVio::gatingTest(const MatrixXd& H, const VectorXd& r, const int& dof) {

    if(H.isZero(0)) return false; // check matrix empty

    MatrixXd P1 = H * state_server.state_cov * H.transpose();
    MatrixXd P2 = Feature::observation_noise * MatrixXd::Identity(H.rows(), H.rows());
    double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

    if (gamma < chi_squared_test_table[dof]) {
        //cout << "passed" << endl;
        return true;
    } else {
        //cout << "failed" << endl;
        return false;
    }
}

void MsckfVio::removeLostFeatures() {
    // Remove the features that lost track.
    // BTW, find the size the final Jacobian matrix and residual vector.
    int jacobian_row_size = 0;
    vector<FeatureIDType> invalid_feature_ids(0);
    vector<FeatureIDType> processed_feature_ids(0);

    for (auto iter = map_server.begin(); iter != map_server.end(); ++iter) {
        // Rename the feature to be checked.
        auto &feature = iter->second;

        // Pass the features that are still being tracked.
        if (feature.observations.find(state_server.imu_state.id) != feature.observations.end())
            continue;
        if (feature.observations.size() < 3) {
            invalid_feature_ids.push_back(feature.id);
            continue;
        }

        // Check if the feature can be initialized if it has not been.
        if (!feature.is_initialized) {
            if (!feature.checkMotion(state_server.cam_states)) {
                invalid_feature_ids.push_back(feature.id);
                continue;
            } else {
                if (!feature.initializePosition(state_server.cam_states)) {
                    invalid_feature_ids.push_back(feature.id);
                    continue;
                }
            }
        }

        jacobian_row_size += 4 * feature.observations.size() - 3;
        processed_feature_ids.push_back(feature.id);
    }

    //cout << "invalid/processed feature #: " << invalid_feature_ids.size() << "/" << processed_feature_ids.size() << endl;
    //cout << "jacobian row #: " << jacobian_row_size << endl;

    // Remove the features that do not have enough measurements.
    for (const auto &feature_id : invalid_feature_ids)
        map_server.erase(feature_id);

    // Return if there is no lost feature to be processed.
    if (processed_feature_ids.empty()) return;

    MatrixXd H_x = MatrixXd::Zero(jacobian_row_size, 21 + 6 * state_server.cam_states.size());
    VectorXd r   = VectorXd::Zero(jacobian_row_size);

    int stack_cntr = 0;

    // Process the features which lose track.
    #pragma omp parallel for
    for (int n=0; n< processed_feature_ids.size(); n++) {

        FeatureIDType feature_id = processed_feature_ids[n];
        auto &feature = map_server[feature_id];

#if WITH_LC
        update_feature(feature); // for loop closure
#endif

        vector<StateIDType> cam_state_ids(0);
        for (const auto &measurement : feature.observations)
            cam_state_ids.push_back(measurement.first);

        MatrixXd H_xj;
        VectorXd r_j;
        featureJacobian(feature.id, cam_state_ids, H_xj, r_j);

        if (gatingTest(H_xj, r_j, cam_state_ids.size() - 1)) {
            H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
            r.segment(stack_cntr, r_j.rows()) = r_j;
            stack_cntr += H_xj.rows();
        }

        // Put an upper bound on the row size of measurement Jacobian, which helps guarantee the executation time.
        if (stack_cntr > 1500)
            break;
    }

    H_x.conservativeResize(stack_cntr, H_x.cols());
    r.conservativeResize(stack_cntr);

#if DEBUG_IMG_J
    if(stack_cntr > 10) {
        cv::Mat Jhx(H_x.rows(), H_x.cols(), CV_8UC1);
        for(int h=0; h<Jhx.rows; h++) for(int w=0; w<Jhx.cols; w++) {
            double val = H_x(h, w);
            Jhx.at<uchar>(h, w) = std::abs(val)>DBL_MIN ? 255 : 0;
        }
        cv::imshow("mat_H01", Jhx);
        cv::waitKey(5);
    }
#endif

    // Perform the measurement update step.
    measurementUpdate(H_x, r);

    // Remove all processed features from the map.
    for (const auto &feature_id : processed_feature_ids)
        map_server.erase(feature_id);

    return;
}

void MsckfVio::findRedundantCamStates(vector<StateIDType>& rm_cam_state_ids) {
    // Move the iterator to the key position.
    auto key_cam_state_iter = state_server.cam_states.end();
    for (int i = 0; i < 4; ++i)
        --key_cam_state_iter;
    auto cam_state_iter = key_cam_state_iter;
    ++cam_state_iter;

    auto first_cam_state_iter = state_server.cam_states.begin();

    // Pose of the key camera state.
    const Vector3d key_position = key_cam_state_iter->second.position;
    const Matrix3d key_rotation = quaternionToRotation(key_cam_state_iter->second.orientation);

    // Mark the camera states to be removed based on the motion between states.
    for (int i = 0; i < 2; ++i) {
        const Vector3d position = cam_state_iter->second.position;
        const Matrix3d rotation = quaternionToRotation(cam_state_iter->second.orientation);

        double distance = (position - key_position).norm();
        double angle = AngleAxisd(rotation * key_rotation.transpose()).angle();

        if (angle < rotation_threshold && distance < translation_threshold && tracking_rate > tracking_rate_threshold) {
            rm_cam_state_ids.push_back(cam_state_iter->first);
            ++cam_state_iter;
        } else {
            rm_cam_state_ids.push_back(first_cam_state_iter->first);
            ++first_cam_state_iter;
        }
    }

    // Sort the elements in the output vector.
    sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

    return;
}

void MsckfVio::pruneCamStateBuffer() {
    if (state_server.cam_states.size() < max_cam_state_size)
        return;

    // Find two camera states to be removed.
    vector<StateIDType> rm_cam_state_ids(0);
    findRedundantCamStates(rm_cam_state_ids);

#if WITH_LC
    StateIDType cam_state_id_margin = std::min(rm_cam_state_ids[0], rm_cam_state_ids[1]);
    cam_state_margin.first = state_server.cam_states.find(cam_state_id_margin)->first;
    cam_state_margin.second = state_server.cam_states.find(cam_state_id_margin)->second;
    is_start_loop = true;
#endif    

    // Find the size of the Jacobian matrix.
    int jacobian_row_size = 0;
    for (auto &item : map_server) {
        auto &feature = item.second;
        // Check how many camera states to be removed are associated with this feature.
        vector<StateIDType> involved_cam_state_ids(0);
        for (const auto &cam_id : rm_cam_state_ids) {
            if (feature.observations.find(cam_id) != feature.observations.end())
                involved_cam_state_ids.push_back(cam_id);
        }

        if (involved_cam_state_ids.size() == 0)
            continue;
        if (involved_cam_state_ids.size() == 1) {
            feature.observations.erase(involved_cam_state_ids[0]);
            continue;
        }

#if WITH_LC
        update_feature(feature); // for loop closure
#endif        

        if (!feature.is_initialized) {
            // Check if the feature can be initialize.
            if (!feature.checkMotion(state_server.cam_states)) {
                // If the feature cannot be initialized, just remove
                // the observations associated with the camera states to be removed.
                for (const auto &cam_id : involved_cam_state_ids)
                    feature.observations.erase(cam_id);
                continue;
            } else {
                if (!feature.initializePosition(state_server.cam_states)) {
                    for (const auto &cam_id : involved_cam_state_ids)
                        feature.observations.erase(cam_id);
                    continue;
                }
            }
        }

        jacobian_row_size += 4 * involved_cam_state_ids.size() - 3;
    }

    //cout << "jacobian row #: " << jacobian_row_size << endl;

    // Compute the Jacobian and residual.
    MatrixXd H_x = MatrixXd::Zero(jacobian_row_size, 21 + 6 * state_server.cam_states.size());
    VectorXd r   = VectorXd::Zero(jacobian_row_size);

    int stack_cntr = 0;

    for (auto &item : map_server) {
        auto &feature = item.second;
        // Check how many camera states to be removed are associated with this feature.
        vector<StateIDType> involved_cam_state_ids(0);
        for (const auto &cam_id : rm_cam_state_ids) {
            if (feature.observations.find(cam_id) != feature.observations.end())
                involved_cam_state_ids.push_back(cam_id);
        }

        if (involved_cam_state_ids.size() == 0)
            continue;

        MatrixXd H_xj;
        VectorXd r_j;
        featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j);

        if (gatingTest(H_xj, r_j, involved_cam_state_ids.size())) {
            H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
            r.segment(stack_cntr, r_j.rows()) = r_j;
            stack_cntr += H_xj.rows();
        }

        for (const auto &cam_id : involved_cam_state_ids)
            feature.observations.erase(cam_id);
    }

    H_x.conservativeResize(stack_cntr, H_x.cols());
    r.conservativeResize(stack_cntr);

#if DEBUG_IMG_J
    if(stack_cntr > 10) {
        cv::Mat Jhx(H_x.rows(), H_x.cols(), CV_8UC1);
        for(int h=0; h<Jhx.rows; h++) for(int w=0; w<Jhx.cols; w++) {
            double val = H_x(h, w);
            Jhx.at<uchar>(h, w) = std::abs(val)>DBL_MIN ? 255 : 0;
        }
        cv::imshow("mat_H02", Jhx);
        cv::waitKey(5);
    }
#endif

    // Perform measurement update.
    measurementUpdate(H_x, r);

    for (const auto &cam_id : rm_cam_state_ids) {
        int cam_sequence = std::distance(state_server.cam_states.begin(), state_server.cam_states.find(cam_id));
        int cam_state_start = 21 + 6 * cam_sequence;
        int cam_state_end = cam_state_start + 6;

        // Remove the corresponding rows and columns in the state covariance matrix.
        if (cam_state_end < state_server.state_cov.rows()) {
            state_server.state_cov.block(cam_state_start, 0, state_server.state_cov.rows() - cam_state_end, state_server.state_cov.cols()) =
                    state_server.state_cov.block(cam_state_end, 0, state_server.state_cov.rows() - cam_state_end, state_server.state_cov.cols());

            state_server.state_cov.block(0, cam_state_start, state_server.state_cov.rows(), state_server.state_cov.cols() - cam_state_end) =
                    state_server.state_cov.block(0, cam_state_end, state_server.state_cov.rows(), state_server.state_cov.cols() - cam_state_end);

            state_server.state_cov.conservativeResize(state_server.state_cov.rows() - 6, state_server.state_cov.cols() - 6);
        } else {
            state_server.state_cov.conservativeResize(state_server.state_cov.rows() - 6, state_server.state_cov.cols() - 6);
        }

        // Remove this camera state in the state vector.
        state_server.cam_states.erase(cam_id);
    }

    return;
}

void MsckfVio::onlineReset() {
    // Never perform online reset if position std threshold is non-positive.
    if (position_std_threshold <= 0)
        return;

    static long long int online_reset_counter = 0;

    // Check the uncertainty of positions to determine if the system can be reset.
    double position_x_std = std::sqrt(state_server.state_cov(12, 12));
    double position_y_std = std::sqrt(state_server.state_cov(13, 13));
    double position_z_std = std::sqrt(state_server.state_cov(14, 14));

    if (position_x_std < position_std_threshold &&
        position_y_std < position_std_threshold &&
        position_z_std < position_std_threshold)
        return;

    ROS_WARN("Start %lld online reset procedure...", ++online_reset_counter);
    ROS_INFO("Stardard deviation in xyz: %f, %f, %f", position_x_std, position_y_std, position_z_std);

    // Remove all existing camera states.
    state_server.cam_states.clear();

    // Clear all exsiting features in the map.
    map_server.clear();

    // Reset the state covariance.
    double gyro_bias_cov, acc_bias_cov, velocity_cov;
    nh.param<double>("initial_covariance/velocity", velocity_cov, 0.25);
    nh.param<double>("initial_covariance/gyro_bias", gyro_bias_cov, 1e-4);
    nh.param<double>("initial_covariance/acc_bias", acc_bias_cov, 1e-2);

    double extrinsic_rotation_cov, extrinsic_translation_cov;
    nh.param<double>("initial_covariance/extrinsic_rotation_cov", extrinsic_rotation_cov, 3.0462e-4);
    nh.param<double>("initial_covariance/extrinsic_translation_cov", extrinsic_translation_cov, 1e-4);

    state_server.state_cov = MatrixXd::Zero(21, 21);
    for (int i = 3; i < 6; ++i)
        state_server.state_cov(i, i) = gyro_bias_cov;
    for (int i = 6; i < 9; ++i)
        state_server.state_cov(i, i) = velocity_cov;
    for (int i = 9; i < 12; ++i)
        state_server.state_cov(i, i) = acc_bias_cov;
    for (int i = 15; i < 18; ++i)
        state_server.state_cov(i, i) = extrinsic_rotation_cov;
    for (int i = 18; i < 21; ++i)
        state_server.state_cov(i, i) = extrinsic_translation_cov;

    ROS_WARN("%lld online reset complete...", online_reset_counter);
    return;
}

#if WITH_LC
void MsckfVio::update_feature(Feature feature) {
    const auto &id = feature.id;

    if(featuresLCDB.find(id) != featuresLCDB.end()) {
        FeatureLcPtr feat = featuresLCDB[id];
        for(const auto &obs : feature.observations) {
            const auto &cam_id = obs.first;
            feat->uvs_norm[cam_id].emplace_back(obs.second);
            Eigen::Vector4d uv = feature.observations_uvs.find(cam_id)->second;
            feat->uvs[cam_id].emplace_back(uv);
            feat->p_FinG = feature.position;
            feat->timestamps[cam_id].emplace_back(feature.timestamp.find(cam_id)->second);            
        }
        return;
    }

    FeatureLcPtr feat = std::make_shared<FeatureLC>();
    feat->featid = id;
    for(const auto &obs : feature.observations) {
        const auto &cam_id = obs.first;
        feat->uvs_norm[cam_id].emplace_back(obs.second);
        Eigen::Vector4d uv = feature.observations_uvs.find(cam_id)->second;
        feat->uvs[cam_id].emplace_back(uv);
        feat->p_FinG = feature.position;
        feat->timestamps[cam_id].emplace_back(feature.timestamp.find(cam_id)->second);
    }
    featuresLCDB.insert({id, feat});
}

void MsckfVio::update_keyframe_historical_information(const std::vector<FeatureLcPtr> &features) {
    // Loop through all features that have been used in the last update
    // We want to record their historical measurements and estimates for later use
    for(const auto &feat : features) {

        // Get position of feature in the global frame of reference
        Eigen::Vector3d p_FinG = feat->p_FinG;

        // // If it is a slam feature, then get its best guess from the state
        // if(state->_features_SLAM.find(feat->featid)!=state->_features_SLAM.end()) {
        //     p_FinG = state->_features_SLAM.at(feat->featid)->get_xyz(false);
        // }

        // Push back any new measurements if we have them
        // Ensure that if the feature is already added, then just append the new measurements
        if(hist_feat_posinG.find(feat->featid)!=hist_feat_posinG.end()) {
            hist_feat_posinG.at(feat->featid) = p_FinG;
            for(const auto &cam2uv : feat->uvs) {
                if(hist_feat_uvs.at(feat->featid).find(cam2uv.first)!=hist_feat_uvs.at(feat->featid).end()) {
                    hist_feat_uvs.at(feat->featid).at(cam2uv.first).insert(hist_feat_uvs.at(feat->featid).at(cam2uv.first).end(), cam2uv.second.begin(), cam2uv.second.end());
                    hist_feat_uvs_norm.at(feat->featid).at(cam2uv.first).insert(hist_feat_uvs_norm.at(feat->featid).at(cam2uv.first).end(), feat->uvs_norm.at(cam2uv.first).begin(), feat->uvs_norm.at(cam2uv.first).end());
                    hist_feat_timestamps.at(feat->featid).at(cam2uv.first).insert(hist_feat_timestamps.at(feat->featid).at(cam2uv.first).end(), feat->timestamps.at(cam2uv.first).begin(), feat->timestamps.at(cam2uv.first).end());
                } else {
                    hist_feat_uvs.at(feat->featid).insert(cam2uv);
                    hist_feat_uvs_norm.at(feat->featid).insert({cam2uv.first,feat->uvs_norm.at(cam2uv.first)});
                    hist_feat_timestamps.at(feat->featid).insert({cam2uv.first,feat->timestamps.at(cam2uv.first)});
                }
            }
        } else {
            hist_feat_posinG.insert({feat->featid,p_FinG});
            hist_feat_uvs.insert({feat->featid,feat->uvs});
            hist_feat_uvs_norm.insert({feat->featid,feat->uvs_norm});
            hist_feat_timestamps.insert({feat->featid,feat->timestamps});
        }
    } 

    std::vector<size_t> ids_to_remove;
    for(const auto &id2feat : hist_feat_timestamps) {
        bool all_older = true;
        for(const auto &cam2time : id2feat.second) {
            for(const auto &time : cam2time.second) {
                if(time >= hist_last_marginalized_time) {
                    all_older = false;
                    break;
                }
            }
            if(!all_older) break;
        }
        if(all_older) {
            ids_to_remove.push_back(id2feat.first);
        }
    }

    // Remove those features!
    for(const auto &id : ids_to_remove) {
        hist_feat_posinG.erase(id);
        hist_feat_uvs.erase(id);
        hist_feat_uvs_norm.erase(id);
        hist_feat_timestamps.erase(id);
    }

    // Remove any historical states older then the marg time
    auto it0 = hist_stateinG.begin();
    while(it0 != hist_stateinG.end()) {
        if(it0->first < hist_last_marginalized_time) it0 = hist_stateinG.erase(it0);
        else it0++;
    }

    // if (state_server.cam_states.size() >= max_cam_state_size)

    if ( is_start_loop ) {
        const auto &cam_state = cam_state_margin.second; // state_server.cam_states.begin()->second;
        hist_last_marginalized_time = cam_state.time;
        assert(hist_last_marginalized_time != INFINITY);
        Eigen::Matrix<double,7,1> state_inG = Eigen::Matrix<double,7,1>::Zero();
        Eigen::Quaterniond q_wc(quaternionToRotation(cam_state.orientation).transpose());
        state_inG(0) = q_wc.x();
        state_inG(1) = q_wc.y();
        state_inG(2) = q_wc.z();
        state_inG(3) = q_wc.w();
        state_inG.tail(3) = cam_state.position;
        hist_stateinG.insert({hist_last_marginalized_time, state_inG});
    } 
}

void MsckfVio::publish_keyframe_information() {
    // if(pub_keyframe_pose.getNumSubscribers()==0 && pub_keyframe_point.getNumSubscribers()==0) return;

    Eigen::Matrix<double,7,1> stateinG;
    if(hist_last_marginalized_time != -1) {
        stateinG = hist_stateinG.at(hist_last_marginalized_time);
    } else {
        stateinG.setZero();
        return;
    }

    printf("[cggos %s] 002 \n", __FUNCTION__);

    std_msgs::Header header;
    header.stamp = ros::Time(hist_last_marginalized_time);

    Isometry3d T_ci = utils::getTransformEigen(nh, "cam0/T_cam_imu");
    Eigen::Vector4d q_ItoC;
    Eigen::Quaterniond q(T_ci.linear().transpose());
    q_ItoC[0] = q.x();
    q_ItoC[1] = q.y();
    q_ItoC[2] = q.z();
    q_ItoC[3] = q.w();
    Eigen::Vector3d p_CinI = - T_ci.linear().transpose() * T_ci.translation();
    nav_msgs::Odometry odometry_calib;
    odometry_calib.header = header;
    odometry_calib.header.frame_id = "imu";
    odometry_calib.pose.pose.position.x = p_CinI(0);
    odometry_calib.pose.pose.position.y = p_CinI(1);
    odometry_calib.pose.pose.position.z = p_CinI(2);
    odometry_calib.pose.pose.orientation.x = q_ItoC(0);
    odometry_calib.pose.pose.orientation.y = q_ItoC(1);
    odometry_calib.pose.pose.orientation.z = q_ItoC(2);
    odometry_calib.pose.pose.orientation.w = q_ItoC(3);
    pub_keyframe_extrinsic.publish(odometry_calib);    

    sensor_msgs::CameraInfo cameraparams;
    cameraparams.header = header;
    cameraparams.header.frame_id = "imu";
    cameraparams.distortion_model = "plumb_bob";
    vector<double> vD(4);
    nh.getParam("cam0/distortion_coeffs", vD);
    cameraparams.D = {vD[0], vD[1], vD[2], vD[3]};
    vector<double> vK(4);
    nh.getParam("cam0/intrinsics", vK);
    double fx = vK[0];
    double fy = vK[1];
    double cx = vK[2];
    double cy = vK[3];    
    cameraparams.K = {fx, 0., cx, 0., fy, cy, 0., 0., 1.};
    pub_keyframe_intrinsics.publish(cameraparams);

    //======================================================
    // PUBLISH HISTORICAL POSE ESTIMATE
    nav_msgs::Odometry odometry_pose;
    odometry_pose.header = header;
    odometry_pose.header.frame_id = "global";
    Eigen::Quaterniond q_wi;
    Eigen::Vector3d t_wi;
    {
        Eigen::Quaterniond q_wc;
        Eigen::Vector3d t_wc;
        q_wc.x() = stateinG(0);
        q_wc.y() = stateinG(1);
        q_wc.z() = stateinG(2);
        q_wc.w() = stateinG(3);
        t_wc = stateinG.tail(3);

        Eigen::Matrix3d r_ci = T_ci.linear();
        Eigen::Vector3d t_ci = T_ci.translation();

        q_wi = q_wc * Eigen::Quaterniond(r_ci);
        t_wi = q_wc.toRotationMatrix() * t_ci + t_wc;
    }
    odometry_pose.pose.pose.position.x = t_wi(0);
    odometry_pose.pose.pose.position.y = t_wi(1);
    odometry_pose.pose.pose.position.z = t_wi(2);
    odometry_pose.pose.pose.orientation.x = q_wi.x();
    odometry_pose.pose.pose.orientation.y = q_wi.y();
    odometry_pose.pose.pose.orientation.z = q_wi.z();
    odometry_pose.pose.pose.orientation.w = q_wi.w();
    pub_keyframe_pose.publish(odometry_pose);   

    printf("[cggos] hist_feat_timestamps size: %d\n", hist_feat_timestamps.size());

    // Construct the message
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = header;
    point_cloud.header.frame_id = "global";
    for(const auto &feattimes : hist_feat_timestamps) {
        StateIDType state_id = cam_state_margin.first;

        // Skip if this feature has no extraction in the "zero" camera
        if(feattimes.second.find(state_id)==feattimes.second.end()) // 0
            continue;

        // printf("[cggos loop] %s, %f, %f, %f \n", __FUNCTION__,
        //     *feattimes.second.at(state_id).begin(),
        //     *(feattimes.second.at(state_id).end()-1),
        //     hist_last_marginalized_time);            

        // Skip if this feature does not have measurement at this time
        auto iter = std::find(feattimes.second.at(state_id).begin(), feattimes.second.at(state_id).end(), hist_last_marginalized_time);
        if(iter==feattimes.second.at(state_id).end())
            continue;

        // Get this feature information
        size_t featid = feattimes.first;
        size_t index = (size_t)std::distance(feattimes.second.at(state_id).begin(), iter);
        Eigen::Vector4d uv = hist_feat_uvs.at(featid).at(state_id).at(index);
        Eigen::Vector4d uv_n = hist_feat_uvs_norm.at(featid).at(state_id).at(index);
        Eigen::Vector3d pFinG = hist_feat_posinG.at(featid);

        // Push back 3d point
        geometry_msgs::Point32 p;
        p.x = pFinG(0);
        p.y = pFinG(1);
        p.z = pFinG(2);
        point_cloud.points.push_back(p);

        // Push back the norm, raw, and feature id
        sensor_msgs::ChannelFloat32 p_2d;
        p_2d.values.push_back(uv_n(0));
        p_2d.values.push_back(uv_n(1));
        p_2d.values.push_back(uv(0));
        p_2d.values.push_back(uv(1));
        p_2d.values.push_back(featid);
        point_cloud.channels.push_back(p_2d);
    }
    pub_keyframe_point.publish(point_cloud);     
}
#endif

void MsckfVio::publish(const ros::Time& time) {
    // Convert the IMU frame to the body frame.
    const IMUState &imu_state = state_server.imu_state;

    Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity(); // Twi
    T_i_w.linear() = quaternionToRotation(imu_state.orientation).transpose();
    T_i_w.translation() = imu_state.position;

    Eigen::Isometry3d T_b_w = IMUState::T_imu_body * T_i_w * IMUState::T_imu_body.inverse();
    Eigen::Vector3d body_velocity = IMUState::T_imu_body.linear() * imu_state.velocity;

    // TUM format
    Eigen::Matrix3d m3_r = T_b_w.rotation();
    Eigen::Vector3d v3_t = T_b_w.translation();
    Eigen::Quaterniond q4_r(m3_r);
    pose_outfile_ << std::fixed << time.toSec() << " "
                  << v3_t[0] << " " << v3_t[1] << " " << v3_t[2] << " "
                  << q4_r.x() << " " << q4_r.y() << " " << q4_r.z() << " " << q4_r.w() << std::endl;

    // Publish tf
    if (publish_tf) {
        tf::Transform T_b_w_tf;
        tf::transformEigenToTF(T_b_w, T_b_w_tf);
        tf_pub.sendTransform(tf::StampedTransform(T_b_w_tf, time, fixed_frame_id, child_frame_id));
    }

    // Publish the odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = fixed_frame_id;
    odom_msg.child_frame_id  = child_frame_id;

    tf::poseEigenToMsg(T_b_w, odom_msg.pose.pose);
    tf::vectorEigenToMsg(body_velocity, odom_msg.twist.twist.linear);

    // Convert the covariance.
    Matrix3d P_oo = state_server.state_cov.block<3, 3>(0, 0);
    Matrix3d P_op = state_server.state_cov.block<3, 3>(0, 12);
    Matrix3d P_po = state_server.state_cov.block<3, 3>(12, 0);
    Matrix3d P_pp = state_server.state_cov.block<3, 3>(12, 12);
    Matrix<double, 6, 6> P_imu_pose = Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;

    Matrix<double, 6, 6> H_pose = Matrix<double, 6, 6>::Zero();
    H_pose.block<3, 3>(0, 0) = IMUState::T_imu_body.linear();
    H_pose.block<3, 3>(3, 3) = IMUState::T_imu_body.linear();
    Matrix<double, 6, 6> P_body_pose = H_pose * P_imu_pose * H_pose.transpose();

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            odom_msg.pose.covariance[6 * i + j] = P_body_pose(i, j);

    // Construct the covariance for the velocity.
    Matrix3d P_imu_vel = state_server.state_cov.block<3, 3>(6, 6);
    Matrix3d H_vel = IMUState::T_imu_body.linear();
    Matrix3d P_body_vel = H_vel * P_imu_vel * H_vel.transpose();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            odom_msg.twist.covariance[i * 6 + j] = P_body_vel(i, j);

    odom_pub.publish(odom_msg);

#if WITH_LC
    // for loop closure
    geometry_msgs::PoseWithCovarianceStamped poseIinM;
    poseIinM.header.stamp = time;
    poseIinM.header.frame_id = fixed_frame_id;
    poseIinM.pose = odom_msg.pose;
    pub_poseimu.publish(poseIinM);
#endif    

    // publish cam odom
    nav_msgs::Odometry odom_cam_msg;
    odom_cam_msg.header = odom_msg.header;
    Eigen::Isometry3d Twc = getCamPose();
    tf::poseEigenToMsg(Twc, odom_cam_msg.pose.pose);
    cam_odom_pub.publish(odom_cam_msg);    

    // Publish the path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = fixed_frame_id;
    pose_stamped.pose = odom_msg.pose.pose;

    path_msg.header.stamp = time;
    path_msg.header.frame_id = fixed_frame_id;
    path_msg.poses.push_back(pose_stamped);
    path_pub.publish(path_msg);

    // Publish the 3D positions of the features that has been initialized.
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_msg_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    feature_msg_ptr->header.frame_id = fixed_frame_id;
    feature_msg_ptr->height = 1;
    for (const auto &item : map_server) {
        const auto &feature = item.second;
        if (feature.is_initialized) {
            Vector3d feature_position = IMUState::T_imu_body.linear() * feature.position;
            feature_msg_ptr->points.push_back(pcl::PointXYZ(feature_position(0), feature_position(1), feature_position(2)));
        }
    }
    feature_msg_ptr->width = feature_msg_ptr->points.size();

    feature_pub.publish(feature_msg_ptr);

    return;
}

} // namespace msckf_vio

