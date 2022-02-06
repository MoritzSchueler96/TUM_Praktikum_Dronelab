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

    /// \brief A Helper struct to send lists of waypoints.
  struct Waypoint {
    double x; ///< The World frame x coordinate.
    double y; ///< The World frame y coordinate.
    double z; ///< The World frame z coordinate.
    double yaw; ///< The yaw angle of the robot w.r.t. the World frame.
    double posTolerance; ///< The position tolerance: if within, it's considered reached.
  };

  /// \brief Plan the trajectory.
   /// \return The value: false means planning failed
  bool plan(Eigen::Vector3d Start,Eigen::Vector3d Goal);
  //Set Occupancy Map
  void setOccupancyMap(cv::Mat MapData);
  /// \brief Load the map
  /// \parameter path The full path to the map file.
  /// \return True on success.
  bool  loadMap(std::string path);

  /// \brief returns Queue of waypoints from A to B
  /// \return std::deque<Waypoint>.
   std::deque<Waypoint> get_waypoints()
   {
     return waypoints_;
   }
   /// \brief returns Queue of waypoints from B to A
  /// \return std::deque<Waypoint>.
   std::deque<Waypoint> get_waypoints_wayback()
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

  /// \brief Reset Path;
  void resetPath();

 protected:
 /// \brief check if given index is occupied
  /// \return true if occupied
 bool is_occupied(int i, int j, int k);
 double calcDist(Eigen::Vector3i Point1, Eigen::Vector3i Point2);
 int getSmallestTotDist(void);
 int NodeinQueue(Eigen::Vector3i Point);
 void addNeighbour(Eigen::Vector3i curPoint, double dist,  Eigen::Vector3i GoalPoint);
  struct Landmark {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Eigen::Vector3d point; ///< The 3d point in World coordinates.
      std::vector<cv::Mat> descriptors; ///< The descriptors: organised one descriptor per row.
  };
  ros::NodeHandle * nh_;  ///< ROS node handle.
  std::atomic<bool> found_; ///< True, if in automatic control mode.

  std::deque<Waypoint> waypoints_wayback;  ///< A list of waypoints that will be approached, if not empty.
  std::deque<Waypoint> waypoints_;  ///< A list of waypoints that will be approached, if not empty.
  std::mutex waypointMutex_;  ///< We need to lock the waypoint access due to asynchronous arrival.
  cv::Mat wrappedMapData_;
  std::map<uint64_t, Landmark, std::less<uint64_t>, 
      Eigen::aligned_allocator<std::pair<const uint64_t, Landmark> > > landmarks_; ///< Landmarks by ID.
  std::deque<Planner::Node> openSet;
};

} // namespace arp


#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_ */
