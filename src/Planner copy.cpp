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


bool Planner::is_occupied(int i, int j, int k)
{
  if(i<wrappedMapData_.size[0]& j<wrappedMapData_.size[1]&k<wrappedMapData_.size[2]&i>=0&j>=0&k>=0)
  {
    
    if(wrappedMapData_.at<char>(i,j,k)>=0)
    {
        return true;
    }
    else
    {
      return false;
    }
  }
  return true;
}
double Planner::calcDist(Eigen::Vector3i Point1, Eigen::Vector3i Point2)
{
  return (Point1-Point2).norm();
}
int Planner::getSmallestTotDist(void)
{
  double minDist=openSet.front().totDistEst;
  int  index=0;
  for(int i=0;i<openSet.size(); i++)
  {
    if(minDist>openSet.at(i).totDistEst)
    {
      minDist=openSet.at(i).totDistEst;
      index=i;
    }

  }
  return index;
}
int Planner::NodeinQueue(Eigen::Vector3i Point)
{
  int index=openSet.size();
  for(int i=0; i< openSet.size();i++)
  {
    if(calcDist(Point,openSet.at(i).point)<1e-3)
    {
      index=i;
      break;
    }

  }
  return index;
}

void Planner::addNeighbour(Eigen::Vector3i curPoint, double dist,  Eigen::Vector3i GoalPoint)
{
  Eigen::Vector3i neighbor;
  neighbor=curPoint;
  for(int x=-1; x<=1; x++)
  {
    for(int y=-1; y<=1; y++)
    {
      for(int z=-1; z<=1; z++)
      {
        neighbor<<curPoint[0]+x,curPoint[1]+y, curPoint[2]+z;
        if(is_occupied(neighbor[0], neighbor[1], neighbor[2]))
        {
          continue;
        }
        double alt=dist+1;
        int index=NodeinQueue(neighbor);
        if(index<openSet.size())
        {
          if(alt<openSet.at(index).dist)
          {
            openSet.at(index).dist=alt;
            openSet.at(index).totDistEst=alt+calcDist(neighbor, GoalPoint);
            openSet.at(index).prev_point=curPoint;
          }
        }
        else
        {
          Node newNode;
          newNode.point=neighbor;
          newNode.dist=alt;
          newNode.totDistEst=alt+calcDist(neighbor, GoalPoint);
          newNode.prev_point=curPoint;
          openSet.push_back(newNode);
        }
      }
    }
  }

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
  openSet.clear();
  //Check if Start and Endpoint are not occupied
  int start_x = std::round(Start[0]/0.1)+(wrappedMapData_.size[0]-1)/2;
  int start_y = std::round(Start[1]/0.1)+(wrappedMapData_.size[1]-1)/2;
  int start_z = std::round(Start[2]/0.1)+(wrappedMapData_.size[2]-1)/2;
  if(is_occupied(start_x, start_y, start_z)) return false;
  int goal_x = std::round(Goal[0]/0.1)+(wrappedMapData_.size[0]-1)/2;
  int goal_y = std::round(Goal[1]/0.1)+(wrappedMapData_.size[1]-1)/2;
  int goal_z = std::round(Goal[2]/0.1)+(wrappedMapData_.size[2]-1)/2;
  if(is_occupied(goal_x, goal_y, goal_z)) return false;
  //check if B is below flight height
  int height=10;//10cm resolution==>1m
  if(goal_z>(start_z+height))
  {
    height=goal_z-start_z+2;
  }
  int flight_height=start_z+10;
  //check if the area above the Goal and Start until flight height is not occupied
  for(int i=1; i<=height; i++)
  {
    if(is_occupied(start_x, start_y, start_z+i))
    {
      //default flight height not possible, set new flight_height
      flight_height=start_z+i-1;
      break;
    }
    if(goal_z+i<=start_z+height)
    {
      if(is_occupied(goal_x, goal_y, goal_z+i))
      {
        //default flight height not possible, set new flight_height
        flight_height=goal_z+i-1;
        break;
      }
    }
    
  }
  //Start point is on floor==> add 1m height
  Node Start_Node;
  Node Goal_Node;
  Start_Node.point<<start_x, start_y, flight_height;
  Goal_Node.point<<goal_x, goal_y, flight_height;
  Start_Node.totDistEst=calcDist(Start_Node.point, Goal_Node.point);
  Start_Node.dist=0;
  Goal_Node.totDistEst=0;
  Goal_Node.dist=0xFFFFFFFFFF;//max value
  openSet.push_back(Start_Node);
  //same for goal
  //find Path in occupancy map with A* in 1m height
  bool finished=false;
  while( openSet.empty()!=true&&finished!=true)
  {
    int index_next_Node= getSmallestTotDist();
    Node Next_Node=openSet.at(index_next_Node);
    openSet.erase(openSet.begin()+index_next_Node);
    //already at goal position?
    if(calcDist(Next_Node.point, Goal_Node.point)<1e-3)
    {
      finished=true;
      continue;
    }
    addNeighbour(Next_Node.point, Next_Node.dist, Goal_Node.point);
  }
    //add to heuristic if neighborhood is occupied

  //for orientation look where the most landmarks are visible

  //for tolerance: look how far next obstacle is
  #endif


    return false;
}


} // namespace arp



