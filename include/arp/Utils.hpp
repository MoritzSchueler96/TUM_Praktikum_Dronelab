#pragma once

#include <queue>

#include <ros/ros.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace arp {

/// Converts a cv::Point3d to an Eigen::Vector3d
inline Eigen::Vector3d cv2ToEigen(const cv::Point3d& vector) {
    return {vector.x, vector.y, vector.z};
}

/// Converts a cv::Point2d to an Eigen::Vector2d
inline Eigen::Vector2d cv2ToEigen(const cv::Point2d& vector) {
    return {vector.x, vector.y};
}

/// Converts a Eigen::Vector3d to a cv::Point3d
inline cv::Point3d eigenToCv2(const Eigen::Vector3d& vector) {
    return {vector.x(), vector.y(), vector.z()};
}

/// Converts a Eigen::Vector2d to a cv::Point2d
inline cv::Point2d eigenToCv2(const Eigen::Vector2d& vector) {
    return {vector.x(), vector.y()};
}

inline double getParamOrFailMiserably(ros::NodeHandle& nh,
                                      const std::string& name) {
    float tmp = 0;
    if (!nh.getParam(name, tmp)) {
        throw std::runtime_error("Getting parameter '" + name +
                                 "' failed miserably");
    }
    return tmp;
}

template<typename T>
inline T getParamOrFailMiserably(ros::NodeHandle& nh,
                                 const std::string& name) {
    T tmp;
    if (!nh.getParam(name, tmp)) {
        throw std::runtime_error("Getting parameter '" + name +
                                 "' failed miserably");
    }
    return tmp;
}

template<typename T>
std::deque<T> concatDeque(const std::deque<T>& a, const std::deque<T>& b) {
    std::deque<T> res;
    for (auto& pt : a) {
        res.push_back(pt);
    }
    for (auto& pt : b) {
        res.push_back(pt);
    }
    return res;
}

template<typename T>
std::string vec2str(const T& vector) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "[" << vector[0] << ", " << vector[1] << ", " << vector[2] << "]";
    return stream.str();
}

}  // namespace arp