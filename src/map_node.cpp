
// #include <fstream>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include<pcl/io/pcd_io.h>
// #include<pcl/point_types.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "arp/MapPublisher.hpp"
#include "arp/Utils.hpp"
#include "arp/OccMap.hpp"


// static std::vector<Eigen::Vector3d> loadMap(std::string path) {
//   std::ifstream mapfile(path);
//   if (!mapfile.good()) {
//     return {};
//   }

//   std::vector<Eigen::Vector3d> points;

//   // read each line
//   std::string line;
//   uint64_t id = 1;
//   while (std::getline(mapfile, line)) {
//     // Convert to stringstream
//     std::stringstream ss(line);

//     Eigen::Vector3d pt;

//     // read 3d position
//     for (int i = 0; i < 3; ++i) {
//       std::string coordString;
//       std::getline(ss, coordString, ',');
//       double coord;
//       std::stringstream(coordString) >> coord;
//       pt[i] = coord;
//     }

//     points.push_back(pt);

//     // Get each descriptor
//     std::string descriptorstring;
//     while (ss.good()) {
//       std::getline(ss, descriptorstring, ',');
//       cv::Mat descriptor(1, 48, CV_8UC1);
//       for (int col = 0; col < 48; ++col) {
//         uint32_t byte;
//         std::stringstream(descriptorstring.substr(2 * col, 2)) >>
//             std::hex >> byte;
//         descriptor.at<uchar>(0, col) = byte;
//       }
//     }
//     id++;
//   }
//   std::cout << "loaded " << points.size() << " landmarks." << std::endl;
//     return points;
// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_node");
    ros::NodeHandle nh;

    auto path = ros::package::getPath("ardrone_practicals");
    auto mapFile = arp::getParamOrFailMiserably<std::string>(nh, "arp_node/occupancymap");
    std::string mapPath = path + "/maps/" + mapFile;
    
    arp::OccMap occMap(mapPath);

    arp::MapPublisher mapPublisher(nh, "occMap");
    auto sub = nh.subscribe("/ardrone/vi_ekf_pose", 1, &arp::MapPublisher::poseCallback, &mapPublisher);

    // ros::Publisher lmPub = nh.advertise<sensor_msgs::PointCloud2>("landmarks", 1, true);

    // std::vector<Eigen::Vector3d> points = loadMap("/home/markus/ardrone_ws/src/ardrone_practicals_2021/maps/okvis2-slam-final_map_skokloster_new2.csv");

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // cloud.width = points.size();
    // cloud.height = 1;
    // for (auto& pt : points) {
    //     cloud.push_back(pcl::PointXYZ(pt[0], pt[1], pt[2]));
    // }

    // sensor_msgs::PointCloud2 cloudMsg;
    // pcl::toROSMsg(cloud, cloudMsg);

    // cloudMsg.header.frame_id = "world";
    // cloudMsg.header.stamp = ros::Time::now();
    // lmPub.publish(cloudMsg);

    while (ros::ok()) {
        ros::spinOnce();

        ros::Duration(0.2).sleep();

        mapPublisher.publish(occMap);
    }

    return 0;
}
