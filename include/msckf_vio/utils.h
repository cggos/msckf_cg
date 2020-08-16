/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_UTILS_H
#define MSCKF_VIO_UTILS_H

#include <ros/ros.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <string>

namespace msckf_vio {

/*
 * @brief utilities for msckf_vio
 */
namespace utils {

Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field);

// boost::math::chi_squared chi_squared_dist(i);
// chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
extern double chi_square_table_p95[99];  // P-value: 0.95, DoF: 1~99,

}  // namespace utils

}  // namespace msckf_vio
#endif
