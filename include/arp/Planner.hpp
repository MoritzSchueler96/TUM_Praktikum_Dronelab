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

namespace arp {

/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Planner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Planner(ros::NodeHandle& nh);

  
  struct Node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;        ///< The 3d point in occupancy grid.
    double totDistEst;            ///< distance to goal
    double dist;                  ///< distance from Start
    Eigen::Vector3d prev_point;   ///< The 3d point in occupancy grid.
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


 protected:
  ros::NodeHandle * nh_;  ///< ROS node handle.
  std::atomic<bool> found_; ///< True, if in automatic control mode.

  std::deque<Waypoint> waypoints_;  ///< A list of waypoints that will be approached, if not empty.
  std::mutex waypointMutex_;  ///< We need to lock the waypoint access due to asynchronous arrival.
  cv::Mat wrappedMapData_;
  std::map<uint64_t, Frontend::Landmark, std::less<uint64_t>, 
      Eigen::aligned_allocator<std::pair<const uint64_t, Frontend::Landmark> > > landmarks_; ///< Landmarks by ID.
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camera_; ///< Camera model
};

} // namespace arp


#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_PLANNER_HPP_ */