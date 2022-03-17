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

#define LANDING_HEIGHT 0.3
#define FLIGHT_HEIGHT 0.7
#define FLIGHT_HEIGHT_INDEX 70
#define STEP_SIZE 0.25
#define POS_TOLERANCE_LAX 0.3
#define POS_TOLERANCE_TIGHT 0.15
#define MAX_TRIES 20
#define ROS_PI 3.141592653589793238462643383279502884L

namespace arp {

Planner::Planner(ros::NodeHandle& nh): nh_(&nh)
{
  //const arp::cameras::RadialTangentialDistortion distortion(cp);
  //arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(cp, distortion);
  //camera_=camMod;
  found_ = false; // always assume no found path  
  isReady_ = false; // always set Planner to not ready
  planningFailed_ = false; // assume planning works till it fails
  calcYawRate_ = true; // calc yaw rate, s.t. camera faces in to the room
  flyForward_ = false; // calc yaw rate, s.t. camera faces forward
  gridSize_ = 5; // default gridsize
  occupancyThres_ = 0; // default threshold
  maxNodesAStar_ = 10000; // default max nodes
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
  //check if index out of given Map
  if(i<wrappedMapData_.size[0]& j<wrappedMapData_.size[1]&k<wrappedMapData_.size[2]&i>=0&j>=0&k>=0)
  {
    //Check Log Odds of Occupancy Map
    if(wrappedMapData_.at<char>(i,j,k)>=occupancyThres_)
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
  //L2 Distance between the points
  return (Point1-Point2).norm();
}

int Planner::getSmallestTotDist(void)
{
  double minDist=openSet.front().totDistEst; //threshold for minimal Total Distance Estiation to goal point
  int  index=0; //return value

  for(int i=0;i<openSet.size(); i++) //go through complete OpenSet
  {
    if(minDist>openSet.at(i).totDistEst) //check if Estimation is lower then Threshold
    {
      minDist=openSet.at(i).totDistEst; //set Threshold to new Minimum
      index=i; //save index to return at the end
    }

  }
  return index;
}

int Planner::NodeinQueue(Eigen::Vector3i Point)
{
  int index=openSet.size();//set index to failure value
  for(int i=0; i< openSet.size();i++) //go through openSet
  {
    if(calcDist(Point,openSet.at(i).point)<1e-3)//check if given point is saed in Node
    {
      index=i;//save index to return
      break; //leave for loop, since point is found
    }

  }
  return index;
}

int Planner::NodeExplored(Eigen::Vector3i Point)
{
  
  int index=exploredSet.size();//set index to failure value
  for(int i=0;i<exploredSet.size();i++)//go through exploredSet
  {
    if(calcDist(Point,exploredSet.at(i).point)<1e-3)//check if given point is saed in Node
    {
      index=i;//save index to return
      break; //leave for loop, since point is found
    }

  }

  return index;
}

void Planner::addNeighbour(Eigen::Vector3i curPoint, double dist,int stepsize, Eigen::Vector3i direction, Eigen::Vector3i GoalPoint)
{
  Eigen::Vector3i neighbor;
  neighbor=curPoint;
  //go through 26 neighbors
  for(int x=-1; x<=1; x++)
  {
    for(int y=-1; y<=1; y++)
    {
      for(int z=-1; z<=1; z++)
      {
        //calculate neighbor with stepsize
        neighbor<<curPoint[0]+x*stepsize,curPoint[1]+y*stepsize, curPoint[2]+z*stepsize; 
        //skip currentpoint, not necessary to stay at same position
        if((neighbor-curPoint).norm()<1e-3)
        {
          continue;
        }
        //if stepsize=1, only 1 point has to be checked
        if(stepsize>1)
        {
          //Check that neighbor no obstacle is between neighbor and current position
          if(!lineCheck(neighbor, curPoint))
          {
            continue;
          }
        }
        else
        {
          //Check that neighbor is not occupied
          if(is_occupied(neighbor[0],neighbor[1], neighbor[2]))
          {
            continue;
          }
        }
        //do not allow to fly over chairs
        if(freetoground(neighbor)==false)
        {
          continue;
        }
        //calculate the covered distance for neighbor
        double alt=dist+(neighbor-curPoint).norm();
        //second value to add preferences, like stay at same direction
        double alttotal=alt;
        //check if point is already explored, so that it is not explored twice, 
        //since distance will be always higher second time
        if(NodeExplored(neighbor)>=exploredSet.size())
        {
          //check if Point is already in OpenSet
          int index=NodeinQueue(neighbor);
          //penalize direction changes
          if(((neighbor-curPoint)-direction).norm()>1e-3)
          {
            alttotal+=2;
          }
 
          if(index<openSet.size())
          {
            //Node is already in OpenSet==> check if it has to be modified
            if(alt<openSet.at(index).dist)
            {
              openSet.at(index).dist=alt;
              openSet.at(index).totDistEst=alttotal+calcDist(neighbor, GoalPoint);
              openSet.at(index).prev_point=curPoint;
            }
          }
          else
          {
            //add Node to OpenSet
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

}

bool Planner::A_Star(Eigen::Vector3i start, Eigen::Vector3i goal)
{
  //Start point is on floor==> add 1m height
  Node Start_Node;
  Node Goal_Node;
  uint32_t count_tries=0;
  Goal_Node.point=goal;
  //Initialise Start Node
  Start_Node.point=start;
  Start_Node.totDistEst=calcDist(Start_Node.point, Goal_Node.point);
  Start_Node.dist=0;
  
  Goal_Node.totDistEst=0;
  Goal_Node.dist=0xFFFFFFFFFF;//max value
  
  //Initialize A Star Algorithm
  exploredSet.clear();
  openSet.clear();
  openSet.push_back(Start_Node);

  bool finished=false;
  while( openSet.empty()!=true&&finished!=true) //&&count_tries<maxNodesAStar_) //run until all Nodes are explored or goal is reached
  {
    count_tries++;
    ROS_WARN_STREAM("Cnt: " << count_tries);
    //get next Node to explore
    int index_next_Node= getSmallestTotDist();
    Node Next_Node=openSet.at(index_next_Node);
    //return Node from openSet and add it to exploredSet
    exploredSet.push_back(Next_Node);
    openSet.erase(openSet.begin()+index_next_Node);
    
    //already at goal position?
    if(calcDist(Next_Node.point, Goal_Node.point)<1e-3)
    {
      finished=true;
      continue;
    }
    //hyperparameter stepsize to speed up algorithm, since resolution of occupancy Map quite high
    int stepsize=10;//gridSize_;
    
    //to reach goal, gradual planning if it is near
    if(calcDist(Next_Node.point, Goal_Node.point)<10)
    {
      stepsize=1;
    }
    //add Neighbor Nodes to openSet
    addNeighbour(Next_Node.point, Next_Node.dist,stepsize,(Next_Node.point-Next_Node.prev_point), Goal_Node.point);
  }

  return finished;
}



bool Planner::plan(Eigen::Vector3d Start,Eigen::Vector3d Goal ){

    waypoints_.clear();
    waypoints_wayback.clear();
    arp::Autopilot::Waypoint temp;
    //Hinweg
    temp.x=Start[0];
    temp.y=Start[1];
    temp.z=FLIGHT_HEIGHT;

    Eigen::Vector3d  direction=(Goal-Start).normalized();
    Eigen::Vector3d defaultdir(1,0,0);

    temp.yaw=acos(direction.transpose()*defaultdir);
    temp.posTolerance=POS_TOLERANCE_LAX;
    waypoints_.push_back(temp);
    temp.x=Goal[0];
    temp.y=Goal[1];
    temp.z=FLIGHT_HEIGHT;

    temp.posTolerance=POS_TOLERANCE_LAX;
    waypoints_.push_back(temp);

    // check if extra landing pos is needed
    checkLandingPos(temp, waypoints_);

    //Rueckweg
    temp.x=Goal[0];
    temp.y=Goal[1];
    temp.z=FLIGHT_HEIGHT;
    //temp.yaw=0;
    temp.posTolerance=POS_TOLERANCE_LAX;
    waypoints_wayback.push_back(temp);
    temp.x=Start[0];
    temp.y=Start[1];
    temp.z=FLIGHT_HEIGHT;
    //temp.yaw=acos(-direction.transpose()*defaultdir);
    temp.posTolerance=POS_TOLERANCE_LAX;
    waypoints_wayback.push_back(temp);

    // check if extra landing pos is needed
    checkLandingPos(temp, waypoints_wayback);

    found_ = true;
    isReady_ = true;
    planningFailed_ = false;

    return true;


}

bool Planner::plan(arp::Autopilot::Waypoint Start,arp::Autopilot::Waypoint Goal){
  waypoints_.clear();
    waypoints_wayback.clear();
    arp::Autopilot::Waypoint temp;
    
    int start_x = std::round(Start.x/0.1)+(wrappedMapData_.size[0]-1)/2;
    int start_y = std::round(Start.y/0.1)+(wrappedMapData_.size[1]-1)/2;
    int start_z = std::round(Start.z/0.1)+(wrappedMapData_.size[2]-1)/2;
    int goal_x = std::round(Goal.x/0.1)+(wrappedMapData_.size[0]-1)/2;
    int goal_y = std::round(Goal.y/0.1)+(wrappedMapData_.size[1]-1)/2;
    int goal_z = std::round(Goal.z/0.1)+(wrappedMapData_.size[2]-1)/2;
    Eigen::Vector3i start_p;
    if(start_z<FLIGHT_HEIGHT_INDEX)
    {
      start_z=FLIGHT_HEIGHT_INDEX;
    }
    start_p<<start_x, start_y, start_z;
    // std::cout << "Start Point "<<start_p[0]<<", "<<start_p[1]<<", "<<start_p[2]<< std::endl;
    
    if(goal_z<FLIGHT_HEIGHT_INDEX)
    {
      goal_z=FLIGHT_HEIGHT_INDEX;
    }
    Eigen::Vector3i goal_p(goal_x, goal_y, goal_z);
    // std::cout << "Goal Point "<<goal_p[0]<<", "<<goal_p[1]<<", "<<goal_p[2]<< std::endl;
    found_=A_Star(start_p, goal_p);
    
    createWaypoints(exploredSet.back());
    
    Goal.z=0.6;
    Start.z=0.6;
    checkLandingPos(waypoints_.back(), waypoints_);
    checkLandingPos(waypoints_wayback.back(), waypoints_wayback);

    if(found_){
      isReady_ = true;
    } else {
      planningFailed_=true;
    }

    return true;


}

void Planner::calcWorldPoint(Eigen::Vector3i point, Eigen::Vector3d& retPoint)
{
  //calculate each coordinate individually
  for (int i=0; i<3;i++)
  {
    retPoint[i]=((double)point[i]-((double)(wrappedMapData_.size[i]-1)/2))*0.1;
  }
  
}
///\brief create Waypoints
void Planner::createWaypoints(Planner::Node StartNode)
{
  arp::Autopilot::Waypoint tempWaypoint;
  Eigen::Vector3d tempPoint;
  Eigen::Vector3d prevPoint;
  Node nextNode=StartNode;
  Node curNode=StartNode;
  Eigen::Vector3i curDirection(0,0,0);
  Eigen::Vector3i prevDirection(0,0,0);
  ROS_INFO_STREAM("Total Distance: "<< StartNode.dist);
  double curYawRate;
  double prevYawRate;
  

  while(nextNode.dist>0)
  {
    int index=NodeExplored(curNode.prev_point);
    nextNode=exploredSet.at(index);
    curDirection=nextNode.point-curNode.point;
    calcWorldPoint(curNode.prev_point, tempPoint);
    calcWorldPoint(curNode.point, tempPoint);

    if(calcYawRate_){
      if(lookFixedPointOrientation_)
      {
        curYawRate=calcYawRate_targetpoint(tempPoint, fixedOrientationPoint_);

      }else{
        curYawRate=calcYawRate_area(tempPoint,prevPoint);
      }

    }
    else curYawRate=0.0;
    if(((curDirection-prevDirection).norm()>1e-3))//||(curYawRate!=prevYawRate))//check if direction changed
    {
      tempWaypoint.x=tempPoint[0];
      tempWaypoint.y=tempPoint[1];
      tempWaypoint.z=tempPoint[2];
      tempWaypoint.yaw=curYawRate;
      tempWaypoint.posTolerance=POS_TOLERANCE_LAX;
      waypoints_.push_front(tempWaypoint);
      tempWaypoint.yaw=0;//yaw rate calculation done later
      waypoints_wayback.push_back(tempWaypoint);
    }

    curNode=nextNode;
    prevDirection=curDirection;
    prevYawRate=curYawRate;
    

  }
  calcWorldPoint(curNode.point, tempPoint);
  tempWaypoint.x=tempPoint[0];
  tempWaypoint.y=tempPoint[1];
  tempWaypoint.z=tempPoint[2];

  if(calcYawRate_) 
  {
    if(lookFixedPointOrientation_)
    {
      tempWaypoint.yaw=calcYawRate_targetpoint(tempPoint, fixedOrientationPoint_);

    }else{
      tempWaypoint.yaw=calcYawRate_area(tempPoint,prevPoint);
    }
  }
  else tempWaypoint.yaw=0.0;
  tempWaypoint.posTolerance=POS_TOLERANCE_LAX;
  waypoints_wayback.push_back(tempWaypoint);


  if(calcYawRate_){
      double yawrate;
    prevPoint<<waypoints_.front().x,waypoints_.front().y,waypoints_.front().z;
    tempPoint<<waypoints_.back().x,waypoints_.back().y,waypoints_.back().z;
    if(lookFixedPointOrientation_)
    {
      yawrate=calcYawRate_targetpoint(tempPoint, fixedOrientationPoint_);
    }else{
      yawrate=calcYawRate_area(tempPoint,prevPoint);
    }
    
    for(int i=0;i<waypoints_.size(); i++)
    {
      std::cout << "yaw rates: " << waypoints_.at(i).yaw;
      tempPoint<<waypoints_.at(i).x,waypoints_.at(i).y,waypoints_.at(i).z;
      if(lookFixedPointOrientation_)
      {
        yawrate=calcYawRate_targetpoint(tempPoint, fixedOrientationPoint_);
      }else{
        yawrate=calcYawRate_area(tempPoint,prevPoint);
      }
      waypoints_.at(i).yaw=yawrate;//calcYawRate_area(tempPoint,prevPoint);
      std::cout << "yaw rates: " << waypoints_.at(i).yaw;
      prevPoint=tempPoint;
    
    }
      //correction of yawrates
      prevPoint<<waypoints_wayback.front().x,waypoints_wayback.front().y,waypoints_wayback.front().z;
      tempPoint<<waypoints_wayback.back().x,waypoints_wayback.back().y,waypoints_wayback.back().z;
      //yawrate=calcYawRate_area(tempPoint,prevPoint);
      for(int i=0;i<waypoints_wayback.size(); i++)
      {
        // std::cout<<"yaw rates"<<waypoints_wayback.at(i).yaw;
        tempPoint<<waypoints_wayback.at(i).x,waypoints_wayback.at(i).y,waypoints_wayback.at(i).z;
        if(lookFixedPointOrientation_)
        {
          yawrate=calcYawRate_targetpoint(tempPoint, fixedOrientationPoint_);
        }else{
          yawrate=calcYawRate_area(tempPoint,prevPoint);
        }
        waypoints_wayback.at(i).yaw=yawrate;//calcYawRate_area(tempPoint,prevPoint);
        // std::cout<<"yaw rates"<<waypoints_wayback.at(i).yaw;
        prevPoint=tempPoint;
      }
  
    
  }

}

void Planner::checkLandingPos(const Eigen::Vector3d pos, std::deque<arp::Autopilot::Waypoint>& waypoints){
    if(pos[2] > LANDING_HEIGHT){
        Eigen::Vector3d landingPos;
        if(!calcLandingPosition(pos, landingPos)) ROS_ERROR("Could not add landing position.");
        else {
            arp::Autopilot::Waypoint landingWaypoint;
            landingWaypoint.x=landingPos[0];
            landingWaypoint.y=landingPos[1];
            landingWaypoint.z=landingPos[2];
            landingWaypoint.yaw=0.0;
            landingWaypoint.posTolerance=POS_TOLERANCE_TIGHT;
            waypoints.push_back(landingWaypoint);
        }
    } else if(pos[2] < LANDING_HEIGHT){
        arp::Autopilot::Waypoint landingWaypoint;
        landingWaypoint.x=pos[0];
        landingWaypoint.y=pos[1];
        landingWaypoint.z=LANDING_HEIGHT;
        landingWaypoint.yaw=0.0;
        landingWaypoint.posTolerance=POS_TOLERANCE_TIGHT;
        std::deque<arp::Autopilot::Waypoint >::iterator it = waypoints.end();
        --it;

        it = waypoints.insert(it,landingWaypoint); 
    }
}

void Planner::checkLandingPos(const arp::Autopilot::Waypoint w, std::deque<arp::Autopilot::Waypoint>& waypoints){
    if(w.z > LANDING_HEIGHT){
        Eigen::Vector3d pos(w.x, w.y, w.z);

        Eigen::Vector3d landingPos;
        if(!calcLandingPosition(pos, landingPos)) ROS_ERROR("Could not add landing position.");
        else {
          arp::Autopilot::Waypoint landingWaypoint;
          landingWaypoint.x=landingPos[0];
          landingWaypoint.y=landingPos[1];
          landingWaypoint.z=landingPos[2];
          landingWaypoint.yaw=w.yaw;
          landingWaypoint.posTolerance=POS_TOLERANCE_TIGHT;
          waypoints.push_back(landingWaypoint);
        }
    } else if(w.z < LANDING_HEIGHT){
          arp::Autopilot::Waypoint landingWaypoint;
          landingWaypoint = w;
          landingWaypoint.z=LANDING_HEIGHT;
          landingWaypoint.posTolerance=POS_TOLERANCE_TIGHT;
          std::deque<arp::Autopilot::Waypoint >::iterator it = waypoints.end();
          --it;

          it = waypoints.insert(it,landingWaypoint); 
    }
}

bool Planner::calcLandingPosition(const Eigen::Vector3d start, Eigen::Vector3d& goal)
{
    ROS_DEBUG("Calculating Landing Position.");
    int mode = 1;
    double step = STEP_SIZE;
    int cnt = 0;

    goal << start;
    goal[2] = LANDING_HEIGHT;

    while(!lineCheck(start, goal) && cnt <= MAX_TRIES){
        goal << start;
        goal[2] = LANDING_HEIGHT;

        switch(mode)
        {
          case 1: 
              goal[0] += step;
              break;
          case 2: 
              goal[1] += step;
              break;
          case 3: 
              goal[0] += step;
              goal[1] += step;
              break;
          case 4:
              goal[0] -= step;
              break;
          case 5:
              goal[1] -= step;
              break;
          case 6:
              goal[0] -= step;
              goal[1] -= step;
              break;
          case 7:
              goal[0] += step;
              goal[1] -= step;
              break;
          case 8:
              goal[0] -= step;
              goal[1] += step;
              step += STEP_SIZE;
              mode = 0;
              cnt++;
              break;
          default:
              ROS_WARN_STREAM("Invalid mode: " << mode);
        }
        mode++;
    }
    ROS_DEBUG_STREAM("Landing Position: " << goal);
    if(cnt >= MAX_TRIES) return false;

    return true;
}

///\brief Check if a obstacle is on estraight line between start and goal position
bool Planner::lineCheck(const Eigen::Vector3d start, const Eigen::Vector3d goal)
{

    int i, j, k;

    Eigen::Vector3d dir;
    dir << (goal - start).normalized();
    Eigen::Vector3d cur;
    cur << start;

    while((cur - start).norm() < (goal-start).norm())
    {
      cur += 0.1*dir;
      i = std::round(cur[0]/0.1)+(wrappedMapData_.size[0]-1)/2;
      j = std::round(cur[1]/0.1)+(wrappedMapData_.size[1]-1)/2;
      k = std::round(cur[2]/0.1)+(wrappedMapData_.size[2]-1)/2;

      if(i<wrappedMapData_.size[0]& j<wrappedMapData_.size[1]&k<wrappedMapData_.size[2]&i>=0&j>=0&k>=0)
      {
        // check if occupied
        if(wrappedMapData_.at<char>(i,j,k)>=occupancyThres_)
        {
            ROS_WARN_THROTTLE(10, "Occupied");
            return false;
        }
      }
      else
      {
        // out of range
        ROS_WARN_THROTTLE(2, "out of range.");
        return false;
      }
    }
    // everything fine
    return true;

}


///\brief Check if a obstacle is on estraight line between start and goal position
bool Planner::lineCheck(const Eigen::Vector3i start, const Eigen::Vector3i goal)
{

    int i, j, k;
    Eigen::Vector3d Start2;
    Eigen::Vector3d Goal2;
    calcWorldPoint(start, Start2);
    calcWorldPoint(goal, Goal2);

    return lineCheck(Start2, Goal2);

}

double Planner::calcYawRate_area(Eigen::Vector3d point, Eigen::Vector3d prev_point)
{
  //Room: -4 -3.5
  //Room: 5 14
  //Table: 2  6
  //Table: 0.6 4
  
  static double prevYawrate=0;
  if (flyForward_)
  {
    Eigen::Vector3d  direction=(prev_point-point).normalized();
    Eigen::Vector3d defaultdir(1,0,0);
    double yawrate=acos(direction.transpose()*defaultdir);
    if(direction[1]<0)
    {

      return yawrate;
    }
    else
    {
      return -yawrate;
    }
  }
  else
  {
    
    if((point[1]<7.5&&point[1]>2.5)&&((point[0]>2)||(point[0]<0.6)))
    {
      if(prev_point[1]>7.5)
      {
        prevYawrate=ROS_PI/2;
          return ROS_PI/2;
      }
      if(prev_point[1]<2.5)
      {
        prevYawrate=-ROS_PI/2;
        return -ROS_PI/2;
      }
      return prevYawrate;
      
    }
    if((point[1]>12)&&(point[0]<4))
    {
      return -ROS_PI/2;
    }
    if((point[1]<-2.5)&&(point[0]<4))
    {
      return -ROS_PI/2;
    }
    if(point[0]>4)
    {
      if(point[1]>5)
      {
        return -ROS_PI*3/4;
      }
      return ROS_PI*3/4;
    }
    return 0;
  }



}


double Planner::calcYawRate_targetpoint(Eigen::Vector3d point, Eigen::Vector3d targetpoint)
{
  Eigen::Vector3d  direction=(targetpoint-point).normalized();
    Eigen::Vector3d defaultdir(1,0,0);
    double yawrate=acos(direction.transpose()*defaultdir);
    if(direction[0]>0)
    {

      return yawrate;
    }
    else
    {
      return -yawrate;
    }

}
 /// \brief check if area below is free
  /// \return true if free
 bool Planner::freetoground(Eigen::Vector3i point)
 {
   Eigen::Vector3i temp;
   //since ground allways occupied add small puffer
   temp<<point[0],point[1],82;
   //std::cout<<"freetognd check"<<temp;
   return lineCheck(point, temp);
 }
} // namespace arp



