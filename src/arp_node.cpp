#include <memory>
#include <unistd.h>
#include <stdlib.h>
#include <fstream>
#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/Planner.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include "arp/cameras/RadialTangentialDistortion.hpp"

#include <arp/Frontend.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <ros/package.h>
#include <ros/console.h>

#include <arp/InteractiveMarkerServer.hpp>

#define FONT_COLOR cv::Scalar(0,255,0)

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(vit_){
      vit_->addImage(timeMicroseconds, lastImage_);
    }
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();   
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void setVIT(arp::VisualInertialTracker& vit){
    vit_ = &vit;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    
    Eigen::Vector3d acc_S;
    acc_S << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    Eigen::Vector3d omega_S;
    omega_S << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    if(vit_){
      vit_->addImuMeasurement(timeMicroseconds, omega_S, acc_S);      
    }
  }

 private:
  cv::Mat lastImage_;
  cv::Mat lastImage_test;
  std::mutex imageMutex_;
  arp::VisualInertialTracker* vit_ = nullptr;
};

  /// \brief Struct holding global variables
struct globalParams{
  // cam parameters
  bool enableFusion;
  bool cameraModelApplied;
  // image overlays
  bool displayKeypoints;   
  bool displayAllKeypoints;
  // image scaling
  int ImageWidth;
  int ImageHeight;
  double fontScaling;
  // pose
  int poseLostThreshold;
  int poseSwitchThreshold;
  ros::Duration poseLostTimeThreshold;
  // planner
  bool calcYawRate;
  bool flyForward;
  bool lookFixedPointOrientation;
  Eigen::Vector3d lookFixedOrientationPoint;
  int gridSize;
  int occupancyThres;
  uint32_t maxNodesAStar;
};

 /// \brief Load global variables
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] gp Struct with global parameters to store values.
  /// @param[out] success Whether Parameters were read successfully.
bool loadGlobalVars(ros::NodeHandle& nh, globalParams& gp){
  if(!nh.getParam("/arp_node/enableFusion", gp.enableFusion)) ROS_FATAL("error loading enableFusion");
  if(!nh.getParam("/arp_node/camModel", gp.cameraModelApplied)) ROS_FATAL("error loading camModel");
  if(!nh.getParam("/arp_node/displayKeypoints", gp.displayKeypoints)) ROS_FATAL("error loading displayKeypoints");
  if(!nh.getParam("/arp_node/ImageWidth", gp.ImageWidth)) ROS_FATAL("error loading ImageWidth");
  if(!nh.getParam("/arp_node/ImageHeight", gp.ImageHeight)) ROS_FATAL("error loading ImageHeight");
  if(!nh.getParam("/arp_node/fontScaling", gp.fontScaling)) ROS_FATAL("error loading fontScaling");
  if(!nh.getParam("/arp_node/displayAllKeypoints", gp.displayAllKeypoints)) ROS_FATAL("error loading displayAllKeypoints");
  if(!nh.getParam("/arp_node/poseLostThreshold", gp.poseLostThreshold)) ROS_FATAL("error loading poseLostThreshold");
  if(!nh.getParam("/arp_node/poseSwitchThreshold", gp.poseSwitchThreshold)) ROS_FATAL("error loading poseSwitchThreshold");

  double threshold;
  if(!nh.getParam("/arp_node/poseLostTimeThreshold", threshold)) ROS_FATAL("error loading poseLostTimeThreshold");
  gp.poseLostTimeThreshold = ros::Duration(threshold);  

  if(!nh.getParam("/arp_node/calcYawRate", gp.calcYawRate)) ROS_FATAL("error loading calcYawRate");
  if(!nh.getParam("/arp_node/flyForward", gp.flyForward)) ROS_FATAL("error loading flyForward");
  if(!nh.getParam("/arp_node/gridSize", gp.gridSize)) ROS_FATAL("error loading gridSize");
  if(!nh.getParam("/arp_node/occupancyThres", gp.occupancyThres)) ROS_FATAL("error loading occupancyThres");

  int maxNodes;
  if(!nh.getParam("/arp_node/maxNodesAStar", maxNodes)) ROS_FATAL("error loading maxNodesAStar");
  gp.maxNodesAStar = (uint32_t) maxNodes;
  if(!nh.getParam("/arp_node/lookFixedPointOrientation", gp.lookFixedPointOrientation)) ROS_FATAL("error loading lookFixedPointOrientation");

  std::vector<double> temp;
  if(!nh.getParam("/arp_node/lookFixedOrientationPoint", temp)) ROS_FATAL("error loading lookFixedOrientationPoint");
  gp.lookFixedOrientationPoint << temp[0], temp[1], temp[2];

  return true;
}

 /// \brief Load PID parameters
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] fp Struct with Frontend parameters to store values.
  /// @param[out] success Whether Parameters were read successfully.
bool loadFrontendParams(ros::NodeHandle& nh, arp::Frontend::frontendParams& fp){
  if(!nh.getParam("/arp_node/skipThresInit", fp.skipThresInit)) ROS_FATAL("error loading skipThresInit");
  if(!nh.getParam("/arp_node/skipThresLimit", fp.skipThresLimit)) ROS_FATAL("error loading skipThresLimit");
  if(!nh.getParam("/arp_node/mapFocalLength", fp.mapFocalLength)) ROS_FATAL("error loading map focal length");
  if(!nh.getParam("/arp_node/Brisk_uniformityRadius", fp.Brisk_uniformityRadius)) ROS_FATAL("error loading Brisk_uniformityRadius");
  if(!nh.getParam("/arp_node/Brisk_absoluteThreshold", fp.Brisk_absoluteThreshold)) ROS_FATAL("error loading Brisk_absoluteThreshold");
  if(!nh.getParam("/arp_node/numKeypoints", fp.numKeypoints)) ROS_FATAL("error loading numKeypoints");
  if(!nh.getParam("/arp_node/distanceThres", fp.distanceThres)) ROS_FATAL("error loading distanceThres");
  if(!nh.getParam("/arp_node/inlierThres", fp.inlierThres)) ROS_FATAL("error loading inlierThres");
  return true;
}

 /// \brief Load PID parameters
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] pp Struct with PID parameters to store values.
  /// @param[out] success Whether Parameters were read successfully.
bool loadPIDParams(ros::NodeHandle& nh, arp::Autopilot::pidParams& pp){
  std::vector<double> temp;
  if(!nh.getParam("/arp_node/xControlParams", temp)) ROS_FATAL("error loading xControlParams");
  pp.xControlParams.k_p = temp[0];
  pp.xControlParams.k_i = temp[1];
  pp.xControlParams.k_d = temp[2];

  if(!nh.getParam("/arp_node/yControlParams", temp)) ROS_FATAL("error loading yControlParams");
  pp.yControlParams.k_p = temp[0];
  pp.yControlParams.k_i = temp[1];
  pp.yControlParams.k_d = temp[2];

  if(!nh.getParam("/arp_node/zControlParams", temp)) ROS_FATAL("error loading zControlParams");
  pp.zControlParams.k_p = temp[0];
  pp.zControlParams.k_i = temp[1];
  pp.zControlParams.k_d = temp[2];

  if(!nh.getParam("/arp_node/yawControlParams", temp)) ROS_FATAL("error loading yawControlParams");
  pp.yawControlParams.k_p = temp[0];
  pp.yawControlParams.k_i = temp[1];
  pp.yawControlParams.k_d = temp[2];

  return true;
}

 /// \brief Read Ransac Parameters from launch file
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] rp Struct to store ransac parameters.
  /// @param[out] success Whether Parameters were read successfully.
bool loadRansacParams(ros::NodeHandle& nh, arp::Frontend::ransacParams& rp){
  if(!nh.getParam("/arp_node/useExtrinsicGuess", rp.useExtrinsicGuess)) ROS_FATAL("error loading useExtrinsicGuess");
  if(!nh.getParam("/arp_node/iterationsCount", rp.iterationsCount)) ROS_FATAL("error loading iterationsCount");
  if(!nh.getParam("/arp_node/reprojectionError", rp.reprojectionError)) ROS_FATAL("error loading reprojectionError");
  if(!nh.getParam("/arp_node/confidence", rp.confidence)) ROS_FATAL("error loading confidence");

  return true;
}

 /// \brief Read Cam Parameters from launch file
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] cp Class Object to store camera parameters.
  /// @param[out] success Whether Parameters were read successfully.
bool readCameraParameters(ros::NodeHandle& nh, arp::cameras::CamParams& cp){

  std::cout << std::setprecision(3) << std::fixed;

  double fu;
  if(!nh.getParam("/arp_node/fu", fu)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter fu...    value=" << fu);

  double fv;
  if(!nh.getParam("/arp_node/fv", fv))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter fv...    value=" << fv);
  
  double cu;
  if(!nh.getParam("/arp_node/cu", cu))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter cu...    value=" << cu);
  
  double cv;
  if(!nh.getParam("/arp_node/cv", cv))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter cv...    value=" << cv);
  
  double k1;
  if(!nh.getParam("/arp_node/k1", k1))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter k1...    value=" << k1);
  
  double k2;
  if(!nh.getParam("/arp_node/k2", k2))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter k2...    value=" << k2);
  
  double p1;
  if(!nh.getParam("/arp_node/p1", p1))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter p1...    value=" << p1);

  double p2;
  if(!nh.getParam("/arp_node/p2", p2))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter p2...    value=" << p2);

  if(!cp.setParams(640, 360, fu, fv, cu, cv, k1, k2, p1, p2)) ROS_FATAL("error setting parameters");
  return true;
}

 /// \brief initialize Cam Model
  /// @param[in] nh NodeHandle.
  /// @param[in] cp Class Object with stored camera parameters.
  /// @param[out] camMod Initialized camera model  
arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> setupCamera(ros::NodeHandle& nh, const arp::cameras::CamParams cp){
  // setup camera model
  const arp::cameras::RadialTangentialDistortion distortion(cp);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(cp, distortion);
  ROS_DEBUG("Initialize undistort maps...");
  if(!camMod.initialiseUndistortMaps(cp)) ROS_FATAL("error initializing map");
  return camMod;
}

 /// \brief initialize Visual Inertial Tracker
  /// @param[in] nh NodeHandle.
  /// @param[in] vit Visual Inertial Tracker Objcet.
  /// @param[in] frontend Frontend Object.
  /// @param[in] viEkf Kalman Filter Object.
  /// @param[in] pubState Publisher Object.
  /// @param[out] success Whether setup was successfull. 
bool setupVisualInertialTracker(ros::NodeHandle& nh, arp::VisualInertialTracker& vit, arp::Frontend& frontend, arp::ViEkf& viEkf, arp::StatePublisher& pubState, arp::Autopilot& autopilot){
  
  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
  ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
  ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  
  // set up EKF
  ROS_DEBUG("Setup EKF...");
  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!nh.getParam("arp_node/T_SC", T_SC_array))
  ROS_FATAL("error loading parameter");
  T_SC_mat <<
  T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
  T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
  T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
  T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  // set up visual-inertial tracking
  ROS_DEBUG("Setup VIT...");
  vit.setFrontend(frontend);
  vit.setEstimator(viEkf);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  vit.setVisualisationCallback(std::bind(
  &arp::StatePublisher::publish, &pubState, std::placeholders::_1,
  std::placeholders::_2));
  vit.setControllerCallback(std::bind(&arp::Autopilot::controllerCallback, &autopilot,
      std::placeholders::_1, std::placeholders::_2));

  return true;
}

 /// \brief initialize Occupancy Map
  /// @param[in] nh NodeHandle.
  /// @param[in] retwrappedMapData Occupancy Map Object.
  /// @param[out] success Whether setup was successfull. 
bool setupOccupancyMap(ros::NodeHandle& nh,cv::Mat& retwrappedMapData)
{
  // open the file:
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFileName;
  if(!nh.getParam("arp_node/occupancymap", mapFileName)) ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFileName;
  std::ifstream mapFile(mapPath, std::ios_base::in | std::ios_base::binary);
  if(!mapFile.is_open()) {
  ROS_FATAL_STREAM("could not open map file " << mapFileName);
  }
  // first read the map size along all the dimensions:
  int sizes[3];
  if(!mapFile.read((char *)sizes, 3* sizeof(int))) {
  ROS_FATAL_STREAM("could not read map file " << mapFileName);
  }
  // now read the map data:
  char * mapData = new char[sizes[0]*sizes[1]*sizes[2]]; // don’t forget \
  to delete[] in the end!
  if(!mapFile.read((char *)mapData, sizes[0]*sizes[1]*sizes[2])) {
  ROS_FATAL_STREAM("could not read map file " << mapFileName);
  }
  mapFile.close();
  // now wrap it with a cv::Mat for easier access:
  cv::Mat wrappedMapData(3,sizes, CV_8SC1, mapData);
  retwrappedMapData=wrappedMapData;

  return true;
}

 /// \brief load waypoints
  /// @param[in] nh NodeHandle.
  /// @param[out] success Whether loading was successfull. 
bool loadWaypoint(ros::NodeHandle& nh, arp::Autopilot::Waypoint& waypoint){
  double x, y, z, yaw, tolerance;

  if(!nh.getParam("/arp_node/waypointX", x)) ROS_FATAL("error loading waypointX");
  if(!nh.getParam("/arp_node/waypointY", y)) ROS_FATAL("error loading waypointY");
  if(!nh.getParam("/arp_node/waypointZ", z)) ROS_FATAL("error loading waypointZ");
  if(!nh.getParam("/arp_node/waypointYaw", yaw)) ROS_FATAL("error loading waypointYaw");
  if(!nh.getParam("/arp_node/waypointTolerance", tolerance)) ROS_FATAL("error loading waypointTolerance");

  waypoint.x = x;
  waypoint.y = y;
  waypoint.z = z;
  waypoint.yaw = yaw;
  waypoint.posTolerance = tolerance;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  /*
  if( ros::console::set_logger_level("ros.ardrone_practicals.debug", ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO_NAMED("custom", "Help");
  */

  // __cplusplus Version
  ROS_INFO_STREAM("Using Cpp Version: " << __cplusplus);

  // read global parameters
  globalParams gp;
  ROS_INFO("Read global parameters...");
  if(!loadGlobalVars(nh, gp)) ROS_FATAL("error loading global variables");

  // load PID parameters
  arp::Autopilot::pidParams pp;
  ROS_INFO("Read PID parameters...");
  if(!loadPIDParams(nh, pp)) ROS_FATAL("error loading PID variables");
 
  // load waypoint
  arp::Autopilot::Waypoint waypointB;
  if(!loadWaypoint(nh, waypointB)) ROS_FATAL("error loading waypoint");

  // read camera parameters
  arp::cameras::CamParams cp;
  ROS_INFO("Read camera parameters...");
  if(!readCameraParameters(nh, cp)) ROS_FATAL("error loading cam parameters");

  // setup camera model
  ROS_INFO("Setup Camera...");
  auto phcam = setupCamera(nh, cp);
  // activate camera model
  ROS_INFO_STREAM("Camera Model Applied set to: " << gp.cameraModelApplied);

  // read frontend parameters
  arp::Frontend::frontendParams fp;
  ROS_INFO("Read frontend parameters...");
  if(!loadFrontendParams(nh, fp)) ROS_FATAL("error loading frontend parameters");

  // read ransac parameters
  arp::Frontend::ransacParams rp;
  ROS_INFO("Read ransac parameters...");
  if(!loadRansacParams(nh, rp)) ROS_FATAL("error loading ransac parameters");  

  // setup frontend
  ROS_INFO("Setup Frontend...");
  std::string map;
  if(!nh.getParam("/arp_node/map", map)) ROS_FATAL("error loading world");
  arp::Frontend frontend(cp, fp);
  frontend.setRansacParams(rp);
  
  //setup OccupancyMap
  ROS_INFO("Setup OccupancyMap...");
  cv::Mat wrappedMapData;
  if(!setupOccupancyMap(nh, wrappedMapData)) ROS_FATAL("error loading occupancy map");
 
  // set up autopilot
  ROS_INFO("Setup Autopilot...");
  arp::Autopilot autopilot(nh, pp);
  autopilot.setOccupancyMap(wrappedMapData);
  int poseLostCnt = 0;

  // set up Planner
  arp::Planner planner(nh);
  planner.setOccupancyMap(wrappedMapData);
  planner.setCalcYawRate(gp.calcYawRate);
  planner.setFlyForward(gp.flyForward);
  planner.setGridSize(gp.gridSize);
  planner.setOccupancyThreshold(gp.occupancyThres);
  planner.setMaxAStarNodes(gp.maxNodesAStar);
  planner.setFixedPointOrientation(gp.lookFixedPointOrientation);
  planner.setFixedOrientationPoint(gp.lookFixedOrientationPoint);
  
  // setup visual inertial tracker
  arp::ViEkf viEkf;
  arp::VisualInertialTracker vit;
  arp::StatePublisher pubState(nh); // state publisher -- provided for rviz visualisation of drone pose:
  ROS_INFO("Setup Visual Inertial Tracker...");
  if(!setupVisualInertialTracker(nh, vit, frontend, viEkf, pubState, autopilot)) ROS_FATAL("error setting up VIT"); 
  bool lastPoseStatus;
  lastPoseStatus = vit.getPoseStatus();
  int poseSwitchCnt=0;
  ros::Time last = ros::Time::now();
  bool poseSwitchTimeUp=false;

  // display keypoints
  frontend.showKeypoints(gp.displayKeypoints);
  ROS_INFO_STREAM("Display Keypoints set to: " << gp.displayKeypoints);
  // display all keypoints
  frontend.showAllKeypoints(gp.displayAllKeypoints);
  ROS_INFO_STREAM("Display all Keypoints set to: " << gp.displayAllKeypoints);
  // enable / disable fusion
  vit.enableFusion(gp.enableFusion);
  ROS_INFO_STREAM("Sensor fusion set to: " << gp.enableFusion);

  // setup inputs
  ROS_INFO("Setup Subscriber...");
  Subscriber subscriber;
  subscriber.setVIT(vit);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);
  
  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, gp.ImageWidth, gp.ImageHeight, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // to get reliable button presses
  bool pressed = false;

  // set challenge parameters
  bool flyChallenge = false;
  bool flyBack = false;
  bool challengeCompleted = false;
  bool challengePaused = false;

  // set Timings for challenge
  ros::Time landingTime = ros::Time::now() + ros::Duration(24*60*60);
  ros::Time challengePauseTime = ros::Time::now() + ros::Duration(24*60*60);
  double challengeStartTime = (ros::Time::now() + ros::Duration(24*60*60)).toSec();
  double challengeTime = 999.99;
  double bestTime = 999.99;

  // start position for challenge
  Eigen::Vector3d challengeStartPos;
  arp::Autopilot::Waypoint challengeStartWaypoint;

  // variables for frontend error messages
  ros::Time displayTime = ros::Time::now() + ros::Duration(24*60*60);
  ros::Time displayTimeChallengeCompleted = ros::Time::now() + ros::Duration(24*60*60);
  std::string errorText = "";

  //create Interactive Marker
  arp::InteractiveMarkerServer markerServer(autopilot);

  // enter main event loop
  std::cout << "" << std::endl;
  std::cout << "================= Hello AR Drone =================" << std::endl;
  std::cout << "" << std::endl;
  cv::Mat image;
  while (ros::ok()) {
      ros::spinOnce();
      ros::Duration dur(0.04);
      dur.sleep();
      SDL_PollEvent(&event);
      if (event.type == SDL_QUIT) {
        break;
      }

      //Multiple Key Capture Begins
      const Uint8 *state = SDL_GetKeyboardState(NULL);

      // check states!
      auto droneStatus = autopilot.droneStatus();
      auto droneBattery=autopilot.droneBattery();

      // render image, if there is a new one available
      if(vit.getLastVisualisationImage(image)) // subscriber.getLastImage(image)
      {
          // TODO: add overlays to the cv::Mat image, e.g. text
          // apply camera model if enabled
          if(gp.cameraModelApplied && !phcam.undistortImage(image, image)) ROS_WARN("Warning: Undistortion failed...");

          // resize image
          cv::resize(image, image,cv::Size(gp.ImageWidth, gp.ImageHeight), CV_INTER_CUBIC);
          cv::Size image_size= image.size();

          // generate Text for drone state and add it to picture
          std::string display="state: "+std::to_string(droneStatus);
          cv::putText(image, display, cv::Point(10*gp.fontScaling, 50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling*2, FONT_COLOR, 2, false); //putText( image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)
          
          // creates text of Batterie charge in 0.1% and add it to picture
          std::stringstream stream;
          stream<< std::fixed<<std::setprecision(1)<<droneBattery <<"%";
          auto color = FONT_COLOR;

          // change font color to red if low battery
          if(droneBattery<10){
              color= cv::Scalar(0,0,255);
          }
          cv::putText(image, stream.str(), cv::Point(image_size.width-200*gp.fontScaling, 50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling*2, color, 2, false);
          
          // put error Text on image
          if(ros::Time::now() - displayTime < ros::Duration(3)) cv::putText(image, errorText, cv::Point(image_size.width/2-350*gp.fontScaling, image_size.height/2), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, cv::Scalar(0,0,255), 2, false);

          // possible commands in buttom of picture, differentiate: drone is flying or not
          // show commands to command drone with rviz
          if(!autopilot.isAutomatic() && !autopilot.isTracking())
          {
              cv::putText(image, "STRG-R: Switch to Auto Mode", cv::Point(image_size.width/2-240*gp.fontScaling, image_size.height-90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          } else {
              cv::putText(image, "Space: Switch to Man. Mode", cv::Point(image_size.width/2-240*gp.fontScaling, image_size.height-90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }

          // show Challenge Status 
          if(ros::Time::now() - displayTimeChallengeCompleted < ros::Duration(3) && challengeCompleted) cv::putText(image, "Challenge Completed! Yay!", cv::Point(image_size.width/2-240*gp.fontScaling, image_size.height/2), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, cv::Scalar(0,0,255), 2, false);
          if(ros::Time::now() - displayTimeChallengeCompleted < ros::Duration(3) && ros::Time::now() - displayTimeChallengeCompleted >= ros::Duration(0) && !challengeCompleted) cv::putText(image, "Challenge Aborted!", cv::Point(image_size.width/2-185*gp.fontScaling, image_size.height/2), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, cv::Scalar(0,0,255), 2, false);

          // show commands to interact with challenge
          if(autopilot.isTracking()){
                challengeTime = ros::Time::now().toSec() - challengeStartTime;
                cv::putText(image, "Challenge ongoing...", cv::Point(10*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          } else if(!autopilot.isAutomatic()) {
            if(challengePaused) cv::putText(image, "P: Resume Challenge", cv::Point(10*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
            else cv::putText(image, "P: Start Challenge", cv::Point(10*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          } else {
                cv::putText(image, "P: Need man. Mode", cv::Point(10*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }

          // display timings for Challenge
          if(challengeTime < bestTime && challengeCompleted){
              bestTime = challengeTime;
              ROS_WARN_STREAM("New BestTime: " << bestTime);
          } 
          std::stringstream best;
          best << std::fixed<<std::setprecision(2) << bestTime << "s";
          cv::putText(image, "best: ", cv::Point(10*gp.fontScaling, 90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          cv::putText(image, best.str(), cv::Point(40*gp.fontScaling, 125*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);

          std::stringstream time;
          time << std::fixed<<std::setprecision(2) << challengeTime << "s";
          cv::putText(image, "time: ", cv::Point(image_size.width-200*gp.fontScaling, 90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          cv::putText(image, time.str(), cv::Point(image_size.width-170*gp.fontScaling, 125*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);

          // show some more commands
          if (gp.enableFusion)
          {
              cv::putText(image, "F: Sensor Fusion is On", cv::Point(image_size.width/2-185*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          } else {
              cv::putText(image, "F: Sensor Fusion is Off", cv::Point(image_size.width/2-185*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }
          cv::putText(image, "K: Toggle Keypoints", cv::Point(10*gp.fontScaling, image_size.height-10*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          cv::putText(image, "M: Toggle Projection", cv::Point(image_size.width-370*gp.fontScaling, image_size.height-10*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          cv::putText(image, "U: Log Position", cv::Point(image_size.width-370*gp.fontScaling, image_size.height-50*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          
          // show possible moving commands
          if(droneStatus==3||droneStatus==4||droneStatus==7) {

              cv::putText(image, "W/ S: up/ down", cv::Point(10*gp.fontScaling, image_size.height-130*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
              cv::putText(image, "A/ D: yaw left/ right", cv::Point(10*gp.fontScaling, image_size.height-90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
              cv::putText(image, "^/ v: for-/ backward", cv::Point(image_size.width-370*gp.fontScaling, image_size.height-130*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
              cv::putText(image, "</ >: left/ right", cv::Point(image_size.width-370*gp.fontScaling, image_size.height-90*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
              cv::putText(image, "L: Landing; ESC: Stop", cv::Point(image_size.width/2-185*gp.fontScaling, image_size.height-10*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }
          else if(droneStatus==2)
          {
              cv::putText(image, "T: Taking off; ESC: Stop", cv::Point(image_size.width/2-185*gp.fontScaling, image_size.height-10*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }
          else
          {
              cv::putText(image, "ESC: Stop", cv::Point(image_size.width/2-80*gp.fontScaling, image_size.height-10*gp.fontScaling), cv::FONT_HERSHEY_SIMPLEX,gp.fontScaling, FONT_COLOR, 2, false);
          }

          // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
          // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
          // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
          // remember to pick the right SDL_PIXELFORMAT_* !
          texture = SDL_CreateTexture(
                  renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
          SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
          SDL_RenderClear(renderer);
          SDL_RenderCopy(renderer, texture, NULL, NULL);
          SDL_RenderPresent(renderer);
          // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
          SDL_DestroyTexture(texture);
      }

      // command
      if (state[SDL_SCANCODE_ESCAPE]) {
        std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus << std::endl;
        if(!autopilot.estopReset()) ROS_WARN("Warning: Shutdown failed...");
      }

      if (state[SDL_SCANCODE_T]) {
        ROS_INFO_STREAM("Taking off...                          status=" << droneStatus);
        if (!autopilot.takeoff()) ROS_WARN("Warning: Take Off failed...");
      }

      if (state[SDL_SCANCODE_L]) {
        ROS_INFO_STREAM("Going to land...                       status=" << droneStatus);
        if (!autopilot.land()) ROS_WARN("Warning: Landing failed...");
      }

      if (state[SDL_SCANCODE_C]) {
        ROS_INFO_STREAM("Requesting flattrim calibration...     status=" << droneStatus);
        if (!autopilot.flattrimCalibrate()) ROS_WARN("Warning: flattrim calibration failed...");
      }

      // log current position when hitting U
      if (state[SDL_SCANCODE_U]){
        double x, y, z, yaw;
        autopilot.getPoseReference(x, y, z, yaw);
        ROS_INFO_STREAM_THROTTLE(1, "Current Position X: " << x);
        ROS_INFO_STREAM_THROTTLE(1, "Current Position Y: " << y);
        ROS_INFO_STREAM_THROTTLE(1, "Current Position Z: " << z);
        ROS_INFO_STREAM_THROTTLE(1, "Current Position Yaw: " << yaw);
      }

      // logic to guarantee safe operation of auto mode, if some restrictions are violated switch to manual mode
      if (autopilot.isAutomatic() && (state[SDL_SCANCODE_SPACE] or droneStatus==2 or (!vit.getPoseStatus() && (poseSwitchTimeUp or poseLostCnt >= gp.poseLostThreshold or poseSwitchCnt >= gp.poseSwitchThreshold)))) {
        poseSwitchCnt=0;
        ROS_INFO_STREAM("Autopilot off...     status=" << droneStatus);
        autopilot.setManual();
        markerServer.deactivate();
        challengePauseTime = ros::Time::now();

        if(droneStatus==2) errorText = "Deavtivate auto mode due to landing the drone.";
        else if(!state[SDL_SCANCODE_SPACE]) errorText = "Deactivate auto mode due to lost pose.";
        if(!state[SDL_SCANCODE_SPACE]) displayTime = ros::Time::now();
      }

      // cnt how often pose is changed
      if(vit.getPoseStatus() != lastPoseStatus){
        // get ros time now
        last = ros::Time::now();
        poseSwitchCnt++;
        ROS_DEBUG_STREAM("Cnt: " << poseSwitchCnt);
        lastPoseStatus = vit.getPoseStatus();
      }

      // count how often pose is lost and how long it's lost
      if(!vit.getPoseStatus() && autopilot.isAutomatic()){
        poseLostCnt++;
        ROS_DEBUG_STREAM("Lost Cnt: " << poseLostCnt);
        if(ros::Time::now() - last > gp.poseLostTimeThreshold && gp.poseLostTimeThreshold != ros::Duration(0)) {
            poseSwitchTimeUp=true;
        } else {
            poseSwitchTimeUp=false;
        }
        ROS_DEBUG_STREAM("time: " << ros::Time::now() - last);
        ROS_DEBUG_STREAM("thres: " << gp.poseLostTimeThreshold);
        ROS_DEBUG_STREAM("poseSwitchTimeUp: " << poseSwitchTimeUp);
      } else {
        poseLostCnt = 0;
      }

      // create errorText if auto mode can't be activated
      if(state[SDL_SCANCODE_RCTRL] && !autopilot.isAutomatic() && (!vit.getPoseStatus() or droneStatus==2)){
        if(droneStatus==2) errorText = "Can't activate auto mode while not flying.";
        else if(!vit.getPoseStatus()) errorText = "Can't activate auto mode while pose not found.";
        displayTime = ros::Time::now();
      }
      
      // Press M to toggle application of camera model
      // Press K to toggle depiction of key points
      // Press F to toggle application of sensor fusion    
      while(SDL_PollEvent(&event))
      {
          if(state[SDL_SCANCODE_M] && pressed){
              gp.cameraModelApplied = gp.cameraModelApplied ^ 1;
              ROS_INFO_STREAM("Toggle Camera Model...  status=" << gp.cameraModelApplied);
              pressed = false;
          } else if(state[SDL_SCANCODE_K] && pressed){
              gp.displayKeypoints = gp.displayKeypoints ^ 1;
              frontend.showKeypoints(gp.displayKeypoints);
              ROS_INFO_STREAM("Toggle Key points...  status=" << gp.displayKeypoints);
              pressed = false;
          } else if(state[SDL_SCANCODE_F] && pressed){
              gp.enableFusion = gp.enableFusion ^ 1;
              vit.enableFusion(gp.enableFusion);
              ROS_INFO_STREAM("Toggle Sensor Fusion...  status=" << gp.enableFusion);
              pressed = false;
          }
          if(state[SDL_SCANCODE_M] || state[SDL_SCANCODE_K] || state[SDL_SCANCODE_F]){
            pressed = true;
          } 
      }

      // trajectory tracking
      if(flyChallenge && autopilot.isTracking()){

       // Takeoff if standing on the ground
        if(droneStatus == 2 && ros::Time::now() - landingTime > ros::Duration(2)) autopilot.takeoff();

        // load path once per challenge
        if(planner.pathFound()){
            ROS_INFO("Load Path");
            autopilot.flyPath(planner.get_waypoints());
            planner.resetPathFound();
        }

        if(planner.planningFailed()){
            ROS_INFO("Planning failed");
            flyChallenge = false;
            flyBack = false;
            challengeCompleted = false;
            challengePaused = false;
            displayTimeChallengeCompleted = ros::Time::now();
            challengeTime = 999.99;
            planner.resetReady();
            autopilot.setManual();
        }
        
        // check progress
        if(autopilot.waypointsLeft() == 0 && planner.isReady() && !planner.pathFound()){

            // land when reached last waypoint
            ROS_INFO("Land.");
            autopilot.land();
            landingTime = ros::Time::now();

            // check if on the way back
            if(flyBack) {
                // completed challenge - reset everything
                ROS_INFO("Challenge Completed");
                flyChallenge = false;
                flyBack = false;
                challengeCompleted = true;
                challengePaused = false;
                displayTimeChallengeCompleted = ros::Time::now();
                planner.resetReady();
                autopilot.setManual();
            } else {
                // command way back
                ROS_INFO("Get back");
                flyBack = true;
                autopilot.flyPath(planner.get_waypoints_wayback());  
            }
        }
      }

      // Pause Challenge by hitting Space
      if(autopilot.isTracking() && state[SDL_SCANCODE_SPACE]){
        ROS_INFO_STREAM("Challenge paused...     status=" << droneStatus);
        flyChallenge = false;        
        challengeCompleted = false;
        challengePaused = true;
        challengePauseTime = ros::Time::now();
        autopilot.setManual();
      }

      // abort Challenge if waited too long or hit the space a second time
      if(challengePaused && !autopilot.isAutomatic() && (ros::Time::now() - challengePauseTime > ros::Duration(20) or (ros::Time::now() - challengePauseTime > ros::Duration(0.5) && state[SDL_SCANCODE_SPACE]))){
          ROS_INFO_STREAM("Challenge aborted...     status=" << droneStatus);
          challengePaused = false;
          challengeCompleted = false;
          flyChallenge = false;
          flyBack = false;
          displayTimeChallengeCompleted = ros::Time::now();
          challengeTime = 999.99;
          planner.resetReady();
          autopilot.setManual();
      }

      // Start Challenge with linear planner when hitting O
      if(state[SDL_SCANCODE_O] && vit.getPoseStatus() && !autopilot.isTracking() && !autopilot.isAutomatic()){
        // activate tracking state and set parameters
        flyChallenge = true;
        challengeCompleted = false;
        markerServer.deactivateFeedback();
        autopilot.setTracking();
        autopilot.takeoff();
        landingTime = ros::Time::now();

        // invoke Planner if challenge not paused
        if(!challengePaused){
            ROS_INFO_STREAM("Start Challenge with linear Planner...     status=" << droneStatus);
            challengeStartTime=ros::Time::now().toSec();

            // get current position
            double x, y, z, yaw;
            autopilot.getPoseReference(x, y, z, yaw);
            
            // set goal position
            challengeStartPos << x,y,z;
            Eigen::Vector3d goal;
            goal << waypointB.x, waypointB.y, waypointB.z;

            // invoke planner
            planner.plan(challengeStartPos, goal);
        } else {
            // resume planner, substract pause time for live timing
            ROS_INFO_STREAM("Resume Challenge...     status=" << droneStatus);
            challengeStartTime += (ros::Time::now() - challengePauseTime).toSec();
            challengePaused = false;
        }
        // reset pause time
        challengePauseTime = ros::Time::now();

      }

      // Start Challenge when hitting P
      if(state[SDL_SCANCODE_P] && vit.getPoseStatus() && !autopilot.isTracking() && !autopilot.isAutomatic()){
        // activate tracking state and set parameters
        flyChallenge = true;
        challengeCompleted = false;
        markerServer.deactivateFeedback();
        autopilot.setTracking();
        autopilot.takeoff();
        landingTime = ros::Time::now();

        // invoke Planner if challenge not paused
        if(!challengePaused){
            ROS_INFO_STREAM("Start Challenge...     status=" << droneStatus);
            challengeStartTime=ros::Time::now().toSec();

            // get current position
            double x, y, z, yaw;
            autopilot.getPoseReference(x, y, z, yaw);

            // set goal position
            challengeStartWaypoint.x = x;
            challengeStartWaypoint.y = y;
            challengeStartWaypoint.z = z;
            challengeStartWaypoint.yaw = yaw;
            challengeStartWaypoint.posTolerance = 0.1;
            
            // invoke planner
            planner.plan(challengeStartWaypoint, waypointB);
        } else {
            // resume planner, substract pause time for live timing
            ROS_INFO_STREAM("Resume Challenge...     status=" << droneStatus);
            challengeStartTime += (ros::Time::now() - challengePauseTime).toSec();
            challengePaused = false;
        }
        // reset pause time
        challengePauseTime = ros::Time::now();
      }

      // TODO: process moving commands when in state 3, 4 or 7
      if((!autopilot.isTracking())&&(!autopilot.isAutomatic())&&(droneStatus==3||droneStatus==4||droneStatus==7)) 
      {
        
          // activate automatic mode to set goal positions with rviz
          if(state[SDL_SCANCODE_RCTRL] && vit.getPoseStatus()) {
            // init safety parameters
            poseSwitchCnt=0;
            poseSwitchTimeUp=false;
            ROS_INFO_STREAM("Autopilot on...     status=" << droneStatus);

            // get current pose
            double x, y, z, yaw;
            autopilot.getPoseReference(x, y, z, yaw);

            // activate auto mode
            markerServer.activate(x, y, z, yaw);
            markerServer.activateFeedback();
            autopilot.setAutomatic();
          }

          double forward=0;
          double left=0;
          double up=0;
          double rotateLeft=0;
          //UP Arrow: move drone forward
          if (state[SDL_SCANCODE_UP]&&!state[SDL_SCANCODE_DOWN]) {
              ROS_DEBUG_NAMED("custom", "Forward ...     ");
              forward+=1;

          }
          //Down Arrow: move drone backwards
          if (state[SDL_SCANCODE_DOWN]&&!state[SDL_SCANCODE_UP]) {
              ROS_DEBUG_NAMED("custom", "Backwards ...   ");
              forward-=1;
          }
          //Right Arrow: move drone right
          if (state[SDL_SCANCODE_RIGHT]&&!state[SDL_SCANCODE_LEFT]) {
              ROS_DEBUG_NAMED("custom", "Right ...       ");
              left-=1;
          }
          //Left Arrow: move drone left
          if (state[SDL_SCANCODE_LEFT]&&!state[SDL_SCANCODE_RIGHT]) {
              ROS_DEBUG_NAMED("custom", "Left ...        ");
              left+=1;
          }
          //'W': move drone Up
          if (state[SDL_SCANCODE_W]&&!state[SDL_SCANCODE_S]) {
              ROS_DEBUG_NAMED("custom", "Up ...          ");
              up+=1;
          }
          //'S': move drone Down
          if (state[SDL_SCANCODE_S]&&!state[SDL_SCANCODE_W]) {
              ROS_DEBUG_NAMED("custom", "Down ...        ");
              up-=1;
          }
          //'A': yaw drone left
          if (state[SDL_SCANCODE_A]&&!state[SDL_SCANCODE_D]) {
              ROS_DEBUG_NAMED("custom", "Rotate Left ... ");
              rotateLeft+=1;
          }
          //'D': yaw drone right
          if (state[SDL_SCANCODE_D]&& !state[SDL_SCANCODE_A]) {
              ROS_DEBUG_NAMED("custom", "Rotate Right ...");
              rotateLeft-=1;
          }
          if(forward==0&&left==0&&up==0&rotateLeft==0) ROS_DEBUG_NAMED("custom", "Hover ...       ");

          std::cout << std::setprecision(2) << std::fixed;
          ROS_DEBUG_STREAM_NAMED("custom", "     status=" << droneStatus);
          ROS_DEBUG_STREAM_NAMED("custom", "...             battery=" << droneBattery);

          //forward moving instructions to Autopilot class
          if (!autopilot.manualMove(forward, left, up, rotateLeft)) ROS_WARN("Warning: Drone Movement failed...");
      }

  }

  // make sure to land the drone...
  bool success = autopilot.land();
  delete &wrappedMapData;
  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

