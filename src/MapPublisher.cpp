
#include "arp/MapPublisher.hpp"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

namespace arp {

MapPublisher::MapPublisher(ros::NodeHandle& nh, const std::string& name) {
    pub_ = nh.advertise<nav_msgs::OccupancyGrid>(name, 1);
}

void MapPublisher::publish(OccMap& map) {
    const int sx = map.sizes()[0];
    const int sy = map.sizes()[1];
    const int sz = map.sizes()[2];

    nav_msgs::OccupancyGrid grid;
    auto offset = map.worldOffset();

    grid.header.frame_id = "world";
    grid.header.stamp = ros::Time::now();

    grid.info.resolution = map.rasterSize();
    grid.info.width = sx;
    grid.info.height = sy;
    grid.info.origin.position.x = offset[0];
    grid.info.origin.position.y = offset[1];
    grid.info.origin.position.z = 0;

    int z = map.toGridPos({0, 0, drone_z_})[2];
    if (z < 0 || z >= sz) {
        z = sz / 2 + 4;
    }

    for (int y = 0; y < sy; y++)
    {
        for (int x = 0; x < sx; x++)    
        {
            grid.data.push_back(map.isOccupiedGrid({x, y, z}) ? 100 : 0);
        }
        
    }

    pub_.publish(grid);
}

void MapPublisher::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_z_ = msg->pose.position.z;
}

}