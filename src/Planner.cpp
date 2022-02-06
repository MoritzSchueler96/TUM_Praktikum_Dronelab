/*
 * Planner.cpp
 *
 *  Created on: 05 Feb 2022
 *      Author: moritzschueler
 */

#include <arp/Planner.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
namespace arp {

Planner::Planner(ros::NodeHandle& nh): nh_(&nh)
{
  //const arp::cameras::RadialTangentialDistortion distortion(cp);
  //arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(cp, distortion);
  //camera_=camMod;
  found_ = false; // always assume no found path  
  isReady_ = false; // always set Planner to not ready

  
}
//Set Occupancy Map
void Planner::setOccupancyMap(cv::Mat MapData)
{
  wrappedMapData_=MapData;
}

  /// \brief Load the map
  /// \parameter path The full path to the map file.
  /// \return True on success.
bool  Planner::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  uint64_t id=1;
  while (std::getline(mapfile, line)) {
    Landmark landmark;

    // Convert to stringstream
    std::stringstream ss(line);

    // read 3d position
    for(int i=0; i<3; ++i) {
      std::string coordString;
      std::getline(ss, coordString, ',');
      double coord;
      std::stringstream(coordString) >> coord;
      landmark.point[i] = coord;
    }

    // Get each descriptor
    std::string descriptorstring;
    while(ss.good()){
      std::getline(ss, descriptorstring, ',');
      cv::Mat descriptor(1,48,CV_8UC1);
      for(int col=0; col<48; ++col) {
        uint32_t byte;
        std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
        descriptor.at<uchar>(0,col) = byte;
      }
      landmark.descriptors.push_back(descriptor);
    }

    // store into map
    landmarks_[id] = landmark;
    id++;
  }
  ROS_INFO_STREAM("Loaded " << landmarks_.size() << " landmarks...");
  return landmarks_.size() > 0;
}

void Planner::resetPath(){
    waypoints_.clear();
    waypoints_wayback.clear();
}


bool Planner::plan(Eigen::Vector3d Start,Eigen::Vector3d Goal ){
    // do shit
  
  #if 1 //Sequence to test further steps
    arp::Autopilot::Waypoint temp;
    //Hinweg
    temp.x=Start[0];
    temp.y=Start[1];
    temp.yaw=0;
    temp.z=Start[2]+1;
    temp.posTolerance=0.5;
    waypoints_.push_back(temp);
    temp.x=Goal[0];
    temp.y=Goal[1];
    temp.z=Goal[2]+1;
    temp.yaw=0;
    temp.posTolerance=0.5;
    waypoints_.push_back(temp);
    temp.z=Goal[2]+0.1;
    temp.posTolerance=0.1;
    waypoints_.push_back(temp);

    //Rueckweg
    temp.x=Goal[0];
    temp.y=Goal[1];
    temp.yaw=0;
    temp.z=Goal[2]+1;
    temp.posTolerance=0.5;
    waypoints_wayback.push_back(temp);
    temp.x=Start[0];
    temp.y=Start[1];
    temp.z=Start[2]+1;
    temp.yaw=0;
    temp.posTolerance=0.5;
    waypoints_wayback.push_back(temp);
    temp.z=Start[2]+0.1;
    temp.posTolerance=0.1;
    waypoints_wayback.push_back(temp);

    found_ = true;
    isReady_ = true;

    return true;
  #else
std::vector<Planner::Node> openSet;
  //Start point is on floor==> add 1m height
  int i = std::round(Start[0]/0.1)+(wrappedMapData_.size[0]-1)/2;
  int j = std::round(Start[1]/0.1)+(wrappedMapData_.size[1]-1)/2;
  int k = std::round(Start[2]/0.1)+(wrappedMapData_.size[2]-1)/2;
  //same for goal
  //find Path in occupancy map with A* in 1m height
    //add to heuristic if neighborhood is occupied

  //for orientation look where the most landmarks are visible

  //for tolerance: look how far next obstacle is
  #endif


    
}

} // namespace arp



