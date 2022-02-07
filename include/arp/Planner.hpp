/*
 * Planner.hpp
 *
 *  Created on: 05 Feb 2022
 *      Author: moritzschueler
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_

#include <mutex>
#include <Eigen/Core>
#include <atomic>
#include <deque>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ros/console.h>

#include <arp/kinematics/Imu.hpp>
#include <arp/PidController.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <arp/Frontend.hpp>
#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>


namespace arp {


/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Planner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Planner(ros::NodeHandle& nh);

  
  struct Node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3i point;        ///< The 3d point in occupancy grid.
    double totDistEst;            ///< distance to goal
    double dist;                  ///< distance from Start
    Eigen::Vector3i prev_point;   ///< The 3d point in occupancy grid.
  };



  /// \brief Plan the trajectory.
   /// \return The value: false means planning failed
  bool plan(Eigen::Vector3d Start,Eigen::Vector3d Goal);
  bool plan(arp::Autopilot::Waypoint Start,arp::Autopilot::Waypoint Goal );

  //Set Occupancy Map
  void setOccupancyMap(cv::Mat MapData);
  /// \brief Load the map
  /// \parameter path The full path to the map file.
  /// \return True on success.
  bool  loadMap(std::string path);

  /// \brief returns Queue of waypoints from A to B
  /// \return std::deque<Waypoint>.
   std::deque<arp::Autopilot::Waypoint> get_waypoints()
   {
     return waypoints_;
   }
   /// \brief returns Queue of waypoints from B to A
  /// \return std::deque<Waypoint>.
   std::deque<arp::Autopilot::Waypoint> get_waypoints_wayback()
   {
     return waypoints_wayback;
   }

  /// \brief Is Planner ready?;
  bool isReady() { return isReady_; }

  /// \brief Has Planner found a path?;
  bool pathFound() { return found_; }

  /// \brief Reset Path found;
  void resetPathFound() { found_=false; }

  /// \brief Reset isReady;
  void resetReady() { isReady_=false; }

  ///\brief create Waypoints
  void createWaypoints(Node StartNode);

  ///\brief Check if a obstacle is on a straight line between start and goal position
  bool lineCheck(const Eigen::Vector3d start, const Eigen::Vector3d goal);

  ///\brief Calculate Landing Position
  bool calcLandingPosition(Eigen::Vector3d& start, Eigen::Vector3d& goal);

 protected:
 /// \brief check if given index is occupied
  /// \return true if occupied
 bool is_occupied(int i, int j, int k);
 double calcDist(Eigen::Vector3i Point1, Eigen::Vector3i Point2);
 int getSmallestTotDist(void);
 int NodeinQueue(Eigen::Vector3i Point);
 void addNeighbour(Eigen::Vector3i curPoint, double dist,  Eigen::Vector3i GoalPoint);
 bool A_Star(Eigen::Vector3i start, Eigen::Vector3i goal);
  struct Landmark {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Eigen::Vector3d point; ///< The 3d point in World coordinates.
      std::vector<cv::Mat> descriptors; ///< The descriptors: organised one descriptor per row.
  };
  ros::NodeHandle * nh_;  ///< ROS node handle.
  std::atomic<bool> found_; ///< True, if in path found.
  std::atomic<bool> isReady_; ///< True, if in planner ready.

  std::deque<arp::Autopilot::Waypoint> waypoints_wayback;  ///< A list of waypoints that will be approached, if not empty.
  std::deque<arp::Autopilot::Waypoint> waypoints_;  ///< A list of waypoints that will be approached, if not empty.
  std::mutex waypointMutex_;  ///< We need to lock the waypoint access due to asynchronous arrival.
  cv::Mat wrappedMapData_;
  std::map<uint64_t, Landmark, std::less<uint64_t>, 
      Eigen::aligned_allocator<std::pair<const uint64_t, Landmark> > > landmarks_; ///< Landmarks by ID.
  std::deque<Planner::Node> openSet;
  std::deque<Planner::Node> exploredSet;
};

} // namespace arp


#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_ */
