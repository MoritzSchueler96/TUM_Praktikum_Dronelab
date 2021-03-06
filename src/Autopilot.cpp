/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <ros/console.h>

#include <arp/kinematics/operators.hpp>
#define ROS_PI 3.141592653589793238462643383279502884L
namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh,  Autopilot::pidParams pp)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  
  isTracking_ = false; // always start in manual mode
  occupancyThres_ = 0; // default threshold
  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  // publisher for moving function
  pubMove_= nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  //setup PID controllers
  //get Drone Limits
  if(!nh.getParam("/ardrone_driver/euler_angle_max", x_y_limit)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter euler_angle_max...    value=" << x_y_limit);
  if(!nh.getParam("/ardrone_driver/control_vz_max", z_limit)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter control_vz_max...    value=" << z_limit);
  if(!nh.getParam("/ardrone_driver/control_yaw", yaw_limit)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter control_yaw...    value=" << yaw_limit);
  
  z_limit=z_limit/1000;//convert from mm/s to m/s

  //x
  x_pid.setParameters(pp.xControlParams);
  x_pid.setOutputLimits(-x_y_limit,x_y_limit);
  x_pid.resetIntegrator();

  //y
  y_pid.setParameters(pp.yControlParams);
  y_pid.setOutputLimits(-x_y_limit,x_y_limit);
  y_pid.resetIntegrator();

  //z
  z_pid.setParameters(pp.zControlParams);
  z_pid.setOutputLimits(-z_limit,z_limit);
  z_pid.resetIntegrator();

  // yaw
  yaw_pid.setParameters(pp.yawControlParams);
  yaw_pid.setOutputLimits(-yaw_limit,yaw_limit);
  yaw_pid.resetIntegrator();
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get the remaining charge of the drone's betterie.
float Autopilot::droneBattery()
{
    ardrone_autonomy::Navdata navdata;
    {
        std::lock_guard<std::mutex> l(navdataMutex_);
        navdata = lastNavdata_;
    }
    return (navdata.batteryPercent);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  islanding=false;
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    islanding=true;
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  if (isAutomatic_)
  {
    return false;
  }
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  // TODO: implement...
    geometry_msgs::Twist moveMsg;
    if (abs(forward)>1||abs(left)>1||abs(up)>1||abs(rotateLeft)>1)
    {
        return false;
    }
    moveMsg.linear.x=forward;
    moveMsg.linear.y=left;
    moveMsg.linear.z=up;
    moveMsg.angular.z=rotateLeft;
    pubMove_.publish(moveMsg);
  return true;
}

// Set to manual control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
  isTracking_ = false;
}

// Set to automatic control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
  isTracking_ = false;
}

// Set to tracking control mode.
void Autopilot::setTracking()
{
  isAutomatic_ = false;
  isTracking_ = true;
}

//Set Occupancy Map
void Autopilot::setOccupancyMap(cv::Mat MapData)
{
  wrappedMapData_=MapData;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  int i,j,k;
  bool not_occupied=true;
  
  if (!isAutomatic_) {
      ref_x_ = x;
      ref_y_ = y;
      ref_z_ = z;
      ref_yaw_ = yaw;
    return true;
  }

  //Ziel-Start und normieren
  //in 0.1m schritten in Map checken ob frei
  Eigen::Vector3d goal;
  goal << x, y, z;
  ROS_DEBUG_STREAM("goal: " << goal);

  Eigen::Vector3d start;
  start << ref_x_,ref_y_,ref_z_;
  ROS_DEBUG_STREAM("start: " << start);

  Eigen::Vector3d cur;
  cur << ref_x_,ref_y_,ref_z_;
  ROS_DEBUG_STREAM("cur: " << cur);

  Eigen::Vector3d last;
  last = cur;

  Eigen::Vector3d dir;
  dir << (goal - start).normalized();
  ROS_DEBUG_STREAM("dir: " << dir);

  while((cur - start).norm() < (goal-start).norm())
  {
      cur += 0.1*dir;
      i = std::round(cur[0]/0.1)+(wrappedMapData_.size[0]-1)/2;
      j = std::round(cur[1]/0.1)+(wrappedMapData_.size[1]-1)/2;
      k = std::round(cur[2]/0.1)+(wrappedMapData_.size[2]-1)/2;
      if(i<wrappedMapData_.size[0]& j<wrappedMapData_.size[1]&k<wrappedMapData_.size[2]&i>=0&j>=0&k>=0)
      {
        
        if(wrappedMapData_.at<char>(i,j,k)>=occupancyThres_)
        {
            goal = last;
            ROS_WARN_THROTTLE(10, "Wall in front.");
            not_occupied=false;
            break;
        }
      }
      else
      {
        ROS_WARN_THROTTLE(10, "Index out of range");
        goal=last;
        not_occupied=false;
        break;
      }
      
      last = cur;
  }

  ROS_DEBUG("Done.");
  ROS_DEBUG_STREAM("Goal:"<<goal);
  ref_x_ = goal[0];
  ref_y_ = goal[1];
  ref_z_ = goal[2];
  ref_yaw_ = yaw;
  return not_occupied;

}

// Get the pose reference.
bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  // only do anything here, if automatic
  if (!isAutomatic_ && !isTracking_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    x_pid.resetIntegrator();
    y_pid.resetIntegrator();
    z_pid.resetIntegrator();
    yaw_pid.resetIntegrator();
    return;
  }

  // TODO: only enable when in flight
  DroneStatus status = droneStatus();
  if(status>2&&status<8&&islanding==false)
  {
    Eigen::Vector3d error_(0,0,0);
    // Get waypoint list, if available
    std::lock_guard<std::mutex> l(waypointMutex_);
    if(!waypoints_.empty() && isTracking_) {
        // TODO: setPoseReference() from current waypoint
        Waypoint w = waypoints_.front();
        ROS_INFO_STREAM_THROTTLE(2, "Current waypointX: " << w.x);
        ROS_INFO_STREAM_THROTTLE(2, "Current waypointY: " << w.y);
        ROS_INFO_STREAM_THROTTLE(2, "Current waypointZ: " << w.z);
        markerServer_.activate(w.x, w.y, w.z, w.yaw);
        setPoseReference(w.x, w.y, w.z, w.yaw);

        // TODO: remove the current waypoint, if the position error is below \
        the tolerance.
        Eigen::Vector3d referencePose_waypoint(w.x, w.y, w.z);
        error_<< referencePose_waypoint-x.t_WS;
        if(abs(error_.norm()) < w.posTolerance) waypoints_.pop_front();
    } else {
    
        //TODO:calculate error
        Eigen::Vector3d referencePose(ref_x_,ref_y_, ref_z_);
        error_<< referencePose-x.t_WS;
    }
    //x.q_WS.toRotationMatrix().transpose() = R_SW
    error_=x.q_WS.toRotationMatrix().transpose()*error_;
    double yaw_error= ref_yaw_-arp::kinematics::yawAngle(x.q_WS);

    //TODO: boundaries of yaw angle
    if(yaw_error>ROS_PI)
    {
      yaw_error=ROS_PI;
    }else if(yaw_error<-ROS_PI)
    {
      yaw_error=-ROS_PI;
    }

    Eigen::Vector3d e_dot(0,0,0);
    //x.q_WS.toRotationMatrix().transpose() = R_SW
    e_dot=-x.q_WS.toRotationMatrix().transpose()*x.v_W;

    // TODO: get ros parameter
    // is done in constructor and saved to x_y_limit, z_limit and yaw_limit, since values are fixed
    // TODO: compute control output
    double x_move=x_pid.control(timeMicroseconds,error_[0], e_dot[0])/x_y_limit;
    double y_move=y_pid.control(timeMicroseconds,error_[1], e_dot[1])/x_y_limit;
    double z_move=z_pid.control(timeMicroseconds,error_[2], e_dot[2])/z_limit;
    double yaw_move=yaw_pid.control(timeMicroseconds,yaw_error, 0)/yaw_limit;

    // TODO: send to move
    move(x_move,y_move,z_move,yaw_move);
  }

  return;
}


}  // namespace arp

