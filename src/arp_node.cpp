#include <memory>
#include <unistd.h>
#include <stdlib.h>

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
#include <arp/globals.hpp>

#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 960
#define CAM_IMAGE_WIDTH 640
#define CAM_IMAGE_HEIGHT 360
#define FONT_SCALING 1.5
#define FONT_COLOR cv::Scalar(0,255,0)
#define ENABLE_CAM_MODEL true
#define SHOW_KEYPOINTS true
#define ENABLE_FUSION true
#define LOG_LEVEL logDEBUG1 //logRELEASE

// Possible Log Levels: logDEBUG, logERROR, logWARNING, logDEBUG1, logRELEASE, logDEBUG2, logINFO, logDEBUG3

// if want only DEBUG -> if(logLevel == logDEBUG)
// if want debug, warning and error -> if(logLevel == logDEBUG1)
// if want context (like setup camera) -> if(logLevel == logDEBUG2)
// if want additional information -> if(logLevel >= logINFO)
// if want all -> if(logLevel == logDEBUG3)

// set log level
loglevel_e logLevel = LOG_LEVEL;

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
  std::cout << "" << std::endl;
  if(logLevel >= logRELEASE) std::cout << "Read camera parameters..." << std::endl;
  if(!nh.getParam("/arp_node/fu", cp.fu)) ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter fu...    value=" << cp.fu;

  if(!nh.getParam("/arp_node/fv", cp.fv))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter fv...    value=" << cp.fv;
  
  if(!nh.getParam("/arp_node/cu", cp.cu))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter cu...    value=" << cp.cu;
  
  if(!nh.getParam("/arp_node/cv", cp.cv))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter cv...    value=" << cp.cv;
  
  if(!nh.getParam("/arp_node/k1", cp.k1))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter k1...    value=" << cp.k1;
  
  if(!nh.getParam("/arp_node/k2", cp.k2))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter k2...    value=" << cp.k2;
  
  if(!nh.getParam("/arp_node/p1", cp.p1))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter p1...    value=" << cp.p1;

  if(!nh.getParam("/arp_node/p2", cp.p2))ROS_FATAL("error loading parameter");
  if(logLevel >= logINFO) std::cout << "Read parameter p2...    value=" << cp.p2;

  return true;
}

 /// \brief Read Cam Parameter from launch file and initialize Cam Model
  /// @param[in] nh NodeHandle.
  /// @param[in] cp Struct with camera parameters.
  /// @param[out] camMod Initialized camera model  
arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> setupCamera(ros::NodeHandle& nh, const camParams cp){
  // setup camera model
  const arp::cameras::RadialTangentialDistortion distortion(cp.k1, cp.k2, cp.p1, cp.p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv, distortion);
  if(logLevel >= logRELEASE) std::cout << "Setup Camera..." << std::endl;
  if(logLevel >= logINFO) std::cout << "Initialize undistort maps...";
  if(!camMod.initialiseUndistortMaps(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv)){
    ROS_FATAL("error initializing map");
  }
  return camMod;
}

bool setupVisualInertialTracker(ros::NodeHandle& nh, arp::VisualInertialTracker& vit, arp::Frontend& frontend, arp::ViEkf& viEkf, arp::StatePublisher& pubState){
  
  // set up frontend
  if(logLevel >= logINFO) std::cout << "Setup Frontend..." << std::endl;

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
  ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
  ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  
  // set up EKF
  if(logLevel >= logINFO) std::cout << "Setup EKF..." << std::endl;
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
  if(logLevel >= logINFO) std::cout << "Setup VIT..." << std::endl;
  vit.setFrontend(frontend);
  vit.setEstimator(viEkf);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  vit.setVisualisationCallback(std::bind(
  &arp::StatePublisher::publish, &pubState, std::placeholders::_1,
  std::placeholders::_2));

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // read camera parameters
  camParams cp;
  if(!readCameraParameters(nh, cp)){
    ROS_FATAL("error loading parameters");
  } 

  // setup camera model
  auto phcam = setupCamera(nh, cp);
  // activate camera model
  bool cameraModelApplied = ENABLE_CAM_MODEL; 
  
  // setup visual inertial tracker
  arp::Frontend frontend(CAM_IMAGE_WIDTH,CAM_IMAGE_HEIGHT , cp.fu, cp.fv, cp.cu, cp.cv, cp.k1, cp.k2, cp.p1, cp.p2);
  arp::ViEkf viEkf;
  arp::VisualInertialTracker vit;
  arp::StatePublisher pubState(nh); // state publisher -- provided for rviz visualisation of drone pose:
  if(logLevel >= logRELEASE) std::cout << "Setup Visual Inertial Tracker..." << std::endl;
  if(!setupVisualInertialTracker(nh, vit, frontend, viEkf, pubState)){
    ROS_FATAL("error setting up VIT");
  } 

  // display keypoints
  bool displayKeypoints = SHOW_KEYPOINTS;
  frontend.showKeypoints(displayKeypoints);
  if(logLevel >= logINFO) std::cout << "Display Keypoints set to: " << displayKeypoints << std::endl;

  // enable / disable fusion
  bool enableFusion = ENABLE_FUSION;
  vit.enableFusion(enableFusion);
  if(logLevel >= logINFO) std::cout << "Sensor fusion set to: " << enableFusion << std::endl;

  // setup inputs
  if(logLevel >= logINFO) std::cout << "Setup Subscriber..." << std::endl;
  Subscriber subscriber;
  subscriber.setVIT(vit);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);
  
  // set up autopilot
  if(logLevel >= logINFO) std::cout << "Setup Autopilot..." << std::endl;
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, IMAGE_WIDTH, IMAGE_HEIGHT, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // to get reliable button presses
  bool pressed = false;

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
          if(cameraModelApplied && !phcam.undistortImage(image, image))
          {
            if(logLevel >= logWARNING) std::cout << "Warning: Undistortion failed..." << std::endl;
          }
          cv::resize(image, image,cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_INTER_CUBIC);
          cv::Size image_size= image.size();

          //generate Text for drone state and add it to picture
          std::string display="state: "+std::to_string(droneStatus);
          cv::putText(image, display, cv::Point(10*FONT_SCALING, 50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING*2, FONT_COLOR, 2, false); //putText( image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)
          //creates text of Batterie charge in 0.1% and add it to picture
          std::stringstream stream;
          stream<< std::fixed<<std::setprecision(1)<<droneBattery <<"%";
          auto color = FONT_COLOR;
          // change font color to red if low battery
          if(droneBattery<10){
              color= cv::Scalar(0,0,255);
          }
          cv::putText(image, stream.str(), cv::Point(image_size.width-200*FONT_SCALING, 50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING*2, color, 2, false);
          if (enableFusion)
          {
              cv::putText(image, "F: Sensor Fusion On", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          } else {
              cv::putText(image, "F: Sensor Fusion Off", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          }
          cv::putText(image, "K: Toggle Keypoints", cv::Point(10*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
          cv::putText(image, "P: Toggle Projection", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              
          // possible commands in buttom of picture, differentiate: drone is flying or not
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
      if(!autopilot.estopReset()) {
          if(logLevel >= logWARNING) std::cout << "Warning: Shutdown failed..." << std::endl;
      }
    }

    if (state[SDL_SCANCODE_T]) {
      if(logLevel >= logRELEASE) std::cout << "Taking off...                          status=" << droneStatus << std::endl;
      if (!autopilot.takeoff()) {
        if(logLevel >= logWARNING) std::cout << "Warning: Take Off failed..." << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      if(logLevel >= logRELEASE) std::cout << "Going to land...                       status=" << droneStatus << std::endl;
      if (!autopilot.land()) {
        if(logLevel >= logWARNING) std::cout << "Warning: Landing failed..." << std::endl;
      }
    }

    if (state[SDL_SCANCODE_C]) {
      if(logLevel >= logRELEASE) std::cout << "Requesting flattrim calibration...     status=" << droneStatus << std::endl;
      if (!autopilot.flattrimCalibrate()) {
        if(logLevel >= logWARNING) std::cout << "Warning: flattrim calibration failed..." << std::endl;
      }
    }

    // Press P to toggle application of camera model
    // Press K to toggle depiction of key points
    // Press F to toggle application of sensor fusion    
    while(SDL_PollEvent(&event))
    {
        if(state[SDL_SCANCODE_P] && pressed){
            cameraModelApplied = cameraModelApplied ^ 1;
            if(logLevel >= logRELEASE) std::cout << "Toggle Camera Model...  status=" << cameraModelApplied << std::endl;
            pressed = false;
        } else if(state[SDL_SCANCODE_K] && pressed){
            displayKeypoints = displayKeypoints ^ 1;
            frontend.showKeypoints(displayKeypoints);
            if(logLevel >= logRELEASE) std::cout << "Toggle Key points...  status=" << displayKeypoints << std::endl;
            pressed = false;
        } else if(state[SDL_SCANCODE_F] && pressed){
            enableFusion = enableFusion ^ 1;
            vit.enableFusion(enableFusion);
            if(logLevel >= logRELEASE) std::cout << "Toggle Sensor Fusion...  status=" << enableFusion << std::endl;
            pressed = false;
        }
        if(state[SDL_SCANCODE_P] || state[SDL_SCANCODE_K] || state[SDL_SCANCODE_F]){
          pressed = true;
        } 
    }

    // TODO: process moving commands when in state 3,4, or 7
    if(droneStatus==3||droneStatus==4||droneStatus==7) 
    {
        double forward=0;
        double left=0;
        double up=0;
        double rotateLeft=0;
        //UP Arrow: move drone forward
        if (state[SDL_SCANCODE_UP]&&!state[SDL_SCANCODE_DOWN]) {
            forward+=1;
            if(logLevel >= logINFO) std::cout << "Forward ...     ";

        }
        //Down Arrow: move drone backwards
        if (state[SDL_SCANCODE_DOWN]&&!state[SDL_SCANCODE_UP]) {
            if(logLevel >= logINFO) std::cout << "Backwards ...   ";
            forward-=1;
        }
        //Right Arrow: move drone right
        if (state[SDL_SCANCODE_RIGHT]&&!state[SDL_SCANCODE_LEFT]) {
            if(logLevel >= logINFO) std::cout << "Right ...       ";
            left-=1;
        }
        //Left Arrow: move drone left
        if (state[SDL_SCANCODE_LEFT]&&!state[SDL_SCANCODE_RIGHT]) {
            if(logLevel >= logINFO) std::cout << "Left ...        ";
            left+=1;
        }
        //'W': move drone Up
        if (state[SDL_SCANCODE_W]&&!state[SDL_SCANCODE_S]) {
            if(logLevel >= logINFO) std::cout << "Up ...          ";
            up+=1;
        }
        //'S': move drone Down
        if (state[SDL_SCANCODE_S]&&!state[SDL_SCANCODE_W]) {
            if(logLevel >= logINFO) std::cout << "Down ...        ";
            up-=1;
        }
        //'A': yaw drone left
        if (state[SDL_SCANCODE_A]&&!state[SDL_SCANCODE_D]) {
            if(logLevel >= logINFO) std::cout << "Rotate Left ... ";
            rotateLeft+=1;
        }
        //'D': yaw drone right
        if (state[SDL_SCANCODE_D]&& !state[SDL_SCANCODE_A]) {
            if(logLevel >= logINFO) std::cout << "Rotate Right ...";
            rotateLeft-=1;
        }
        if(forward==0&&left==0&&up==0&rotateLeft==0) {
            if(logLevel >= logINFO) std::cout << "Hover ...       ";
        }
        std::cout << std::setprecision(2) << std::fixed;
        if(logLevel >= logINFO) std::cout << "     status=" << droneStatus;
        if(logLevel >= logINFO) std::cout << "...             battery=" << droneBattery << std::endl;
        //forward moving instructions to Autopilot class
        if (!autopilot.manualMove(forward, left, up, rotateLeft)) {
            if(logLevel >= logWARNING) std::cout << "Warning: Drone Movement failed..." << std::endl;
        }
    }

  }


  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

