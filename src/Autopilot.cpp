/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

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


  arp::PidController::Parameters controllerParameters;
  controllerParameters.k_p=0.05;
  controllerParameters.k_i=0;
  controllerParameters.k_d=0;
  x_pid.setParameters(controllerParameters);

  x_pid.setOutputLimits(-x_y_limit,x_y_limit);
  x_pid.resetIntegrator();
  y_pid.setParameters(controllerParameters);
  y_pid.setOutputLimits(-x_y_limit,x_y_limit);
  y_pid.resetIntegrator();
  
  controllerParameters.k_p=1.0;
  controllerParameters.k_i=0;
  controllerParameters.k_d=0;
  z_pid.setParameters(controllerParameters);
  z_pid.setOutputLimits(-z_limit,z_limit);
  z_pid.resetIntegrator();

  controllerParameters.k_p=1.5;
  controllerParameters.k_i=0;
  controllerParameters.k_d=0;
  yaw_pid.setParameters(controllerParameters);
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

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

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
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }

  // TODO: only enable when in flight
  DroneStatus status = droneStatus();
  if(status>2&&status<8)
  {
    //TODO:calculate error
    Eigen::Vector3d referencePose(ref_x_,ref_y_, ref_z_);
    Eigen::Vector3d error_(0,0,0);
    error_<< referencePose-x.t_WS;
    error_=x.q_WS.toRotationMatrix()*error_;
    double yaw_error= ref_yaw_-arp::kinematics::yawAngle(x.q_WS);
    //TODO: bounderies of yaw angle
    if(yaw_error>3.14)
    {
      yaw_error=3.14;
    }else if(yaw_error<-3.14)
    {
      yaw_error=-3.14;
    }

    Eigen::Vector3d e_dot(0,0,0);
    e_dot=-x.q_WS.toRotationMatrix()*x.v_W;


    // TODO: get ros parameter
    //is done in constructor and saved to x_y_limit, z_limit and yaw_limit, since values are fixed
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

