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
#include <arp/cameras/PinholeCamera.hpp>
#include "arp/cameras/RadialTangentialDistortion.hpp"

#include <arp/Frontend.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <ros/package.h>
#include <ros/console.h>

#include <arp/InteractiveMarkerServer.hpp>


#define CAM_IMAGE_WIDTH 640
#define CAM_IMAGE_HEIGHT 360
#define FONT_SCALING 1.5
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
  bool enableFusion;
  bool cameraModelApplied;
  bool displayKeypoints;  
  int numKeypoints;
  double map_focallength;
  double Brisk_uniformityRadius;
  double Brisk_absoluteThreshold;
  int ImageWidth;
  int ImageHeight;
  bool displayAllKeypoints;
  int poseLostThreshold;
  int poseSwitchThreshold;
};

 /// \brief Load global variables
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] gp Struct with global parameters to store values.
  /// @param[out] success Whether Parameters were read successfully.
bool loadGlobalVars(ros::NodeHandle& nh, globalParams& gp){
  if(!nh.getParam("/arp_node/enableFusion", gp.enableFusion)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter fusion...    value=" << gp.enableFusion);

  if(!nh.getParam("/arp_node/camModel", gp.cameraModelApplied)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter cam model...    value=" << gp.cameraModelApplied);

  if(!nh.getParam("/arp_node/displayKeypoints", gp.displayKeypoints)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter display Keypoints...    value=" << gp.displayKeypoints);

  if(!nh.getParam("/arp_node/map_focallength", gp.map_focallength)) ROS_FATAL("error loading map focallength");
  if(!nh.getParam("/arp_node/Brisk_uniformityRadius", gp.Brisk_uniformityRadius)) ROS_FATAL("error loading Brisk_uniformityRadius");
  if(!nh.getParam("/arp_node/Brisk_absoluteThreshold", gp.Brisk_absoluteThreshold)) ROS_FATAL("error loading Brisk_absoluteThreshold");
  if(!nh.getParam("/arp_node/numKeypoints", gp.numKeypoints)) ROS_FATAL("error loading numKeypoints");
  if(!nh.getParam("/arp_node/ImageWidth", gp.ImageWidth)) ROS_FATAL("error loading ImageWidth");
  if(!nh.getParam("/arp_node/ImageHeight", gp.ImageHeight)) ROS_FATAL("error loading ImageHeight");
  if(!nh.getParam("/arp_node/displayAllKeypoints", gp.displayAllKeypoints)) ROS_FATAL("error loading displayAllKeypoints");
  if(!nh.getParam("/arp_node/poseLostThreshold", gp.poseLostThreshold)) ROS_FATAL("error loading poseLostThreshold");
  if(!nh.getParam("/arp_node/poseSwitchThreshold", gp.poseSwitchThreshold)) ROS_FATAL("error loading poseSwitchThreshold");

  return true;
}

 /// \brief Struct holding the camera parameters
struct camParams{
  double fu;
  double fv;
  double cu;
  double cv;
  double k1;
  double k2;
  double p1;
  double p2;
};
 
 /// \brief Read Cam Parameters from launch file
  /// @param[in] nh NodeHandle to read parameters from launch file.
  /// @param[in] cp Struct with camera parameters to store values.
  /// @param[out] success Whether Parameters were read successfully.
bool readCameraParameters(ros::NodeHandle& nh, camParams& cp){

  std::cout << std::setprecision(3) << std::fixed;

  if(!nh.getParam("/arp_node/fu", cp.fu)) ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter fu...    value=" << cp.fu);

  if(!nh.getParam("/arp_node/fv", cp.fv))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter fv...    value=" << cp.fv);
  
  if(!nh.getParam("/arp_node/cu", cp.cu))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter cu...    value=" << cp.cu);
  
  if(!nh.getParam("/arp_node/cv", cp.cv))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter cv...    value=" << cp.cv);
  
  if(!nh.getParam("/arp_node/k1", cp.k1))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter k1...    value=" << cp.k1);
  
  if(!nh.getParam("/arp_node/k2", cp.k2))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter k2...    value=" << cp.k2);
  
  if(!nh.getParam("/arp_node/p1", cp.p1))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter p1...    value=" << cp.p1);

  if(!nh.getParam("/arp_node/p2", cp.p2))ROS_FATAL("error loading parameter");
  ROS_DEBUG_STREAM("Read parameter p2...    value=" << cp.p2);

  return true;
}

 /// \brief initialize Cam Model
  /// @param[in] nh NodeHandle.
  /// @param[in] cp Struct with camera parameters.
  /// @param[out] camMod Initialized camera model  
arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> setupCamera(ros::NodeHandle& nh, const camParams cp){
  // setup camera model
  const arp::cameras::RadialTangentialDistortion distortion(cp.k1, cp.k2, cp.p1, cp.p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv, distortion);
  ROS_DEBUG("Initialize undistort maps...");
  if(!camMod.initialiseUndistortMaps(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv)) ROS_FATAL("error initializing map");
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
 
  // read camera parameters
  camParams cp;
  ROS_INFO("Read camera parameters...");
  if(!readCameraParameters(nh, cp)) ROS_FATAL("error loading parameters");

  // setup camera model
  ROS_INFO("Setup Camera...");
  auto phcam = setupCamera(nh, cp);
  // activate camera model
  ROS_INFO_STREAM("Camera Model Applied set to: " << gp.cameraModelApplied);

  
  // setup frontend
  ROS_INFO("Setup Frontend...");
  std::string map;
  
 
  if(!nh.getParam("/arp_node/map", map)) ROS_FATAL("error loading world");
  arp::Frontend frontend(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv, cp.k1, cp.k2, cp.p1, cp.p2, gp.numKeypoints, gp.map_focallength,gp.Brisk_uniformityRadius ,gp.Brisk_absoluteThreshold);
  //setup OccupancyMap
  ROS_INFO("Setup OccupancyMap...");
  cv::Mat wrappedMapData;
  if(!setupOccupancyMap(nh, wrappedMapData)) ROS_FATAL("error loading occupancy map");
 
  // set up autopilot
  ROS_INFO("Setup Autopilot...");
  arp::Autopilot autopilot(nh);
  autopilot.setOccupancyMap(wrappedMapData);
  int poseLostCnt = 0;
  
  // setup visual inertial tracker
  arp::ViEkf viEkf;
  arp::VisualInertialTracker vit;
  arp::StatePublisher pubState(nh); // state publisher -- provided for rviz visualisation of drone pose:
  ROS_INFO("Setup Visual Inertial Tracker...");
  if(!setupVisualInertialTracker(nh, vit, frontend, viEkf, pubState, autopilot)) ROS_FATAL("error setting up VIT"); 
  bool lastPoseStatus;
  lastPoseStatus = vit.getPoseStatus();
  int poseSwitchCnt=0;

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
  // to get single switch to manual mode and don't flood terminal
  bool changed = true;

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
      if(vit.getLastVisualisationImage(image))
      {
          // TODO: add overlays to the cv::Mat image, e.g. text

          // apply camera model if enabled
          if(gp.cameraModelApplied && !phcam.undistortImage(image, image)) ROS_WARN("Warning: Undistortion failed...");

          // resize image
          cv::resize(image, image,cv::Size(gp.ImageWidth, gp.ImageHeight), CV_INTER_CUBIC);
          cv::Size image_size= image.size();

          // generate Text for drone state and add it to picture
          std::string display="state: "+std::to_string(droneStatus);
          cv::putText(image, display, cv::Point(10*FONT_SCALING, 50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING*2, FONT_COLOR, 2, false); //putText( image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)
          
          // creates text of Batterie charge in 0.1% and add it to picture
          std::stringstream stream;
          stream<< std::fixed<<std::setprecision(1)<<droneBattery <<"%";
          auto color = FONT_COLOR;

          // change font color to red if low battery
          if(droneBattery<10){
              color= cv::Scalar(0,0,255);
          }
          cv::putText(image, stream.str(), cv::Point(image_size.width-200*FONT_SCALING, 50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING*2, color, 2, false);
          
          // possible commands in buttom of picture, differentiate: drone is flying or not
          if(!autopilot.isAutomatic())
          {
              cv::putText(image, "STRG-R: Switch to Auto Mode", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-90*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          } else {
              cv::putText(image, "Space: Switch to Man. Mode", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-90*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          }
          if (gp.enableFusion)
          {
              cv::putText(image, "F: Sensor Fusion is On", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          } else {
              cv::putText(image, "F: Sensor Fusion is Off", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          }
          cv::putText(image, "K: Toggle Keypoints", cv::Point(10*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          cv::putText(image, "P: Toggle Projection", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              
          if(droneStatus==3||droneStatus==4||droneStatus==7) {
              cv::putText(image, "W/ S: up/ down", cv::Point(10*FONT_SCALING, image_size.height-90*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "A/ D: yaw left/ right", cv::Point(10*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "^/ v: for-/ backward", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-90*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "</ >: left/ right", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "L: Landing; ESC: Stop", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          }
          else if(droneStatus==2)
          {
              cv::putText(image, "T: Taking off; ESC: Stop", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          }
          else
          {
              cv::putText(image, "ESC: Stop", cv::Point(image_size.width/2-80*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
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

    //ROS_INFO_STREAM("pose" << vit.getPoseStatus());
    if (autopilot.isAutomatic() && (state[SDL_SCANCODE_SPACE] or (droneStatus==2 && !changed) or (!vit.getPoseStatus() && !changed && poseLostCnt >= gp.poseLostThreshold) or poseSwitchCnt >= gp.poseSwitchThreshold)) {
      changed = true;
      poseSwitchCnt=0;
      ROS_INFO_STREAM("Autopilot off...     status=" << droneStatus);
      autopilot.setManual();
      markerServer.deactivate();
    }

    if(!vit.getPoseStatus() && autopilot.isAutomatic()){
       poseLostCnt++;
      ROS_WARN_STREAM("Lost Cnt: " << poseLostCnt);
    } else {
      poseLostCnt = 0;
    }
    
    if(vit.getPoseStatus() != lastPoseStatus){
      // get ros time now
      poseSwitchCnt++;
      ROS_WARN_STREAM("Cnt: " << poseSwitchCnt);
      lastPoseStatus = vit.getPoseStatus();
    }

    // Press P to toggle application of camera model
    // Press K to toggle depiction of key points
    // Press F to toggle application of sensor fusion    
    while(SDL_PollEvent(&event))
    {
        if(state[SDL_SCANCODE_P] && pressed){
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
        if(state[SDL_SCANCODE_P] || state[SDL_SCANCODE_K] || state[SDL_SCANCODE_F]){
          pressed = true;
        } 
    }

    // TODO: process moving commands when in state 3, 4 or 7
    if((!autopilot.isAutomatic())&&(droneStatus==3||droneStatus==4||droneStatus==7)) 
    {
        
        if (state[SDL_SCANCODE_RCTRL] && vit.getPoseStatus()) {
          changed = false;
          poseSwitchCnt=0;
          ROS_INFO_STREAM("Autopilot on...     status=" << droneStatus);
          double x, y, z, yaw;
          autopilot.getPoseReference(x, y, z, yaw);
          markerServer.activate(x, y, z, yaw);
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

