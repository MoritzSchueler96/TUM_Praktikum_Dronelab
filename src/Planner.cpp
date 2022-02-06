/*
 * Planner.cpp
 *
 *  Created on: 05 Feb 2022
 *      Author: moritzschueler
 */

#include <arp/Planner.hpp>

namespace arp {

Planner::Planner(ros::NodeHandle& nh,arp::cameras::CamParams cp)
    : nh_(&nh)
{
  const arp::cameras::RadialTangentialDistortion distortion(cp);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(cp, distortion);
  camera_=camMod;
  found_ = false; // always assume no found path  
  

  
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
    Frontend::Landmark landmark;

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


bool Planner::plan(Eigen::Vector3d Start,Eigen::Vector3d Goal ){
    // do shit
  std::vector<Planner::Node> openSet;
  //Start point is on floor==> add 1m height
  //same for goal
  //find Path in occupancy map with A* in 1m height

  //add to heuristic if neighborhood is occupied

  //for orientation look where the most landmarks are visible

  //for tolerance: look how far next obstacle is
    return false;
}

} // namespace arp



