/*
 * Autopilot.hpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_

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

#include <arp/WaypointMarkerServer.hpp>
#include <arp/kinematics/Imu.hpp>
#include <arp/PidController.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace arp {

/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Autopilot {
 public:

   /// \brief Struct holding PID parameters
  struct pidParams{
    arp::PidController::Parameters xControlParams;
    arp::PidController::Parameters yControlParams;
    arp::PidController::Parameters zControlParams;
    arp::PidController::Parameters yawControlParams;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Autopilot(ros::NodeHandle& nh, pidParams pp);

  /// \brief These are reverse engineered AR Drone states.
  enum DroneStatus {
    Unknown = 0,
    Inited = 1,
    Landed = 2,
    Flying = 3,
    Hovering = 4,
    Test = 5, // ?
    TakingOff = 6,
    Flying2 = 7,
    Landing = 8,
    Looping = 9 // ?
  };

  /// \brief Get the drone status.
  /// \return The status.
  DroneStatus droneStatus();

  /// \brief Get the remaining charge of the drone’s battery .
   /// \return The value: 0 means no battery, 100 means full battery.
  float droneBattery();

  /// \brief Set to manual control mode.
  void setManual();

  /// \brief Set to automatic control mode.
  void setAutomatic();

  /// \brief Are we currently in automatic mode?;
  bool isAutomatic() { return isAutomatic_; }

  /// \brief set occupancy threshold
  void setOccupancyThreshold(uint8_t value) {occupancyThres_=value;}

  /// \brief Set to tracking control mode.
  void setTracking();

  /// \brief Are we currently in tracking mode?;
  bool isTracking() { return isTracking_; }

  /// \brief Request flattrim calibration.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool flattrimCalibrate();

  /// \brief Takeoff.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool takeoff();

  /// \brief Land.
  /// \return True on success.
  /// \note This will only work when in flight.
  bool land();

  /// \brief Turn off all motors and reboot.
  /// \return True on success.
  /// \warning When in flight, this will let the drone drop down to the ground.
  bool estopReset();

  /// \brief Move the drone manually.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  /// \note This will only do something when in manual mode and flying.
  bool manualMove(double forward, double left, double up, double rotateLeft);

  /// \brief Move the drone automatically.
  /// @param[in] x World x position reference [m].
  /// @param[in] y World y position reference [m].
  /// @param[in] z World z position reference [m].
  /// @param[in] yaw Yaw angle reference [rad].
  /// \return True on success.
  /// \note  This will only do something when in automatic mode and flying.
  bool setPoseReference(double x, double y, double z, double yaw);

  /// \brief Get the pose reference.
  /// @param[out] x World x position reference [m].
  /// @param[out] y World y position reference [m].
  /// @param[out] z World z position reference [m].
  /// @param[out] yaw Yaw angle reference [rad].
  /// \return True on success.
  bool getPoseReference(double& x, double& y, double& z, double& yaw);

  /// \brief The callback from the estimator that sends control outputs to the drone
  /// \note  This will only do something when in automatic mode and flying.
  void controllerCallback(uint64_t timeMicroseconds,
                          const arp::kinematics::RobotState& x);

  /// \brief Set occupancy map.
  void setOccupancyMap(cv::Mat MapData);

  /// \brief A Helper struct to send lists of waypoints.
  struct Waypoint {
    double x; ///< The World frame x coordinate.
    double y; ///< The World frame y coordinate.
    double z; ///< The World frame z coordinate.
    double yaw; ///< The yaw angle of the robot w.r.t. the World frame.
    double posTolerance; ///< The position tolerance: if within, it's considered reached.
  };

  /// \brief Command the drone to fly to these waypoints in order
  ///        (front to back). When finished, the drone will hover at
  ///        the last waypoint.
  /// @param[in] waypoints Waypoint list.
  void flyPath(const std::deque<Waypoint>& waypoints) {
    std::lock_guard<std::mutex> l(waypointMutex_);
    waypoints_ = waypoints;
  }

  /// \brief How many waypoints still have to be flown to?
  /// \return The number of waypoints still not reached.
  int waypointsLeft() {
    std::lock_guard<std::mutex> l(waypointMutex_);
    return waypoints_.size();
  }
  

 protected:
  /// \brief Move the drone.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  bool move(double forward, double left, double up, double rotateLeft);

  /// \brief Obtain the last navdata package (callback).
  /// @param[out] navdata The navdata message.
  void navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::NodeHandle * nh_;  ///< ROS node handle.
  ros::Publisher pubReset_;  ///< The reset publisher -- use to reset the drone (e-stop).
  ros::Publisher pubTakeoff_;  ///< Publish to take-off the drone.
  ros::Publisher pubLand_;  ///< Publish to land the drone.
  ros::ServiceClient srvFlattrim_;  ///< To request a flat trim calibration.
  ardrone_autonomy::Navdata lastNavdata_; ///< Store navdata as it comes in asynchronously.
  std::mutex navdataMutex_; ///< We need to lock navdata access due to asynchronous arrival.
  ros::Subscriber subNavdata_; ///< The subscriber for navdata.

  ros::Publisher pubMove_; ///<publish to cmd_vel
  double ref_x_ = 0.0; ///< World frame x position reference [m].
  double ref_y_ = 0.0; ///< World frame y position reference [m].
  double ref_z_ = 0.0; ///< World frame z position reference [m].
  double ref_yaw_ = 0.0; ///< World frame yaw reference [rad].
  std::mutex refMutex_; ///< We need to lock the reference access due to asynchronous arrival.
  std::atomic<bool> isAutomatic_; ///< True, if in automatic control mode.
  std::atomic<bool> isTracking_; ///< True, if in tracking control mode.
  PidController x_pid; ///< PID Controller for x position.
  PidController y_pid; ///< PID Controller for y position.
  PidController z_pid; ///< PID Controller for z position.
  PidController yaw_pid; ///< PID Controller for yaw rate.
  double x_y_limit; ///< limit for x and y position.
  double z_limit; ///< limit for z position.
  double yaw_limit; ///< limit for yaw rateS.
  cv::Mat wrappedMapData_; ///< Matrix containing occupancy map.
  uint8_t occupancyThres_; ///< Threshold to decide if sth is occupied
  bool islanding;///< flag to indicate that landing is triggered

  std::deque<Waypoint> waypoints_;  ///< A list of waypoints that will be approached, if not empty.
  std::mutex waypointMutex_;  ///< We need to lock the waypoint access due to asynchronous arrival.
  WaypointMarkerServer markerServer_;

};

} // namespace arp


#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_ */
