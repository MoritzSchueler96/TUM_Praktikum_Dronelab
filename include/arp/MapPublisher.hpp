#pragma once

#include <string>

#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "arp/OccMap.hpp"

namespace arp {

class MapPublisher {
    ros::Publisher pub_;
    double drone_z_ = 0;

public:
    MapPublisher(ros::NodeHandle& nh, const std::string& name);

    void publish(OccMap& map);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

}
