/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <msckf_vio/utils.h>

#include <vector>

namespace msckf_vio {
namespace utils {

Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field) {
    Eigen::Isometry3d T;
    cv::Mat c = getTransformCV(nh, field);

    T.linear()(0, 0) = c.at<double>(0, 0);
    T.linear()(0, 1) = c.at<double>(0, 1);
    T.linear()(0, 2) = c.at<double>(0, 2);
    T.linear()(1, 0) = c.at<double>(1, 0);
    T.linear()(1, 1) = c.at<double>(1, 1);
    T.linear()(1, 2) = c.at<double>(1, 2);
    T.linear()(2, 0) = c.at<double>(2, 0);
    T.linear()(2, 1) = c.at<double>(2, 1);
    T.linear()(2, 2) = c.at<double>(2, 2);
    T.translation()(0) = c.at<double>(0, 3);
    T.translation()(1) = c.at<double>(1, 3);
    T.translation()(2) = c.at<double>(2, 3);
    return T;
}

cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field) {
    cv::Mat T;
    try {
        // first try reading kalibr format
        T = getKalibrStyleTransform(nh, field);
    } catch (std::runtime_error &e) {
        // maybe it's the old style format?
        ROS_DEBUG_STREAM("cannot read transform " << field << " in kalibr format, trying old one!");
        try {
            T = getVec16Transform(nh, field);
        } catch (std::runtime_error &e) {
            std::string msg = "cannot read transform " + field + " error: " + e.what();
            ROS_ERROR_STREAM(msg);
            throw std::runtime_error(msg);
        }
    }
    return T;
}

cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field) {
    std::vector<double> v;
    nh.getParam(field, v);
    if (v.size() != 16) {
        throw std::runtime_error("invalid vec16!");
    }
    cv::Mat T = cv::Mat(v).clone().reshape(1, 4);  // one channel 4 rows
    return T;
}

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
    XmlRpc::XmlRpcValue lines;
    if (!nh.getParam(field, lines)) {
        throw(std::runtime_error("cannot find transform " + field));
    }
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw(std::runtime_error("invalid transform " + field));
    }
    for (int i = 0; i < lines.size(); i++) {
        if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw(std::runtime_error("bad line for transform " + field));
        }
        for (int j = 0; j < lines[i].size(); j++) {
            if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                throw(std::runtime_error("bad value for transform " + field));
            } else {
                T.at<double>(i, j) = static_cast<double>(lines[i][j]);
            }
        }
    }
    return T;
}

double chi_square_table_p95[99] = {
    0.00393, 0.10259, 0.35185, 0.71072, 1.14548,
    1.63538, 2.16735, 2.73264, 3.32511, 3.94030,
    4.57481, 5.22603, 5.89186, 6.57063, 7.26094,
    7.96165, 8.67176, 9.39046, 10.11701, 10.85081,
    11.59131, 12.33801, 13.09051, 13.84843, 14.61141,
    15.37916, 16.15140, 16.92788, 17.70837, 18.49266,
    19.28057, 20.07191, 20.86653, 21.66428, 22.46502,
    23.26861, 24.07494, 24.88390, 25.69539, 26.50930,
    27.32555, 28.14405, 28.96472, 29.78748, 30.61226,
    31.43900, 32.26762, 33.09808, 33.93031, 34.76425,
    35.59986, 36.43709, 37.27589, 38.11622, 38.95803,
    39.80128, 40.64593, 41.49195, 42.33931, 43.18796,
    44.03787, 44.88902, 45.74138, 46.59491, 47.44958,
    48.30538, 49.16227, 50.02023, 50.87924, 51.73928,
    52.60031, 53.46233, 54.32531, 55.18923, 56.05407,
    56.91982, 57.78645, 58.65394, 59.52229, 60.39148,
    61.26148, 62.13229, 63.00389, 63.87626, 64.74940,
    65.62328, 66.49790, 67.37323, 68.24928, 69.12603,
    70.00346, 70.88157, 71.76034, 72.63977, 73.51984,
    74.40054, 75.28186, 76.16379, 77.04633};

}  // namespace utils

}  // namespace msckf_vio
