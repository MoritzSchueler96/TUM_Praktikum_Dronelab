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

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 360
#define CAM_IMAGE_WIDTH 640
#define CAM_IMAGE_HEIGHT 360
#define FONT_SCALING 1.5
#define FONT_COLOR cv::Scalar(0,255,0)
#define CAM_MODEL true
#define ENABLE_FUSION true

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
    static int i=0;
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    i++;
    if(vit_)
    {
      cv::Mat test;
      if(vit_->getLastVisualisationImage(test))
      {
        std::cout << " [ vit image ]"<<i<< std::endl;
        i=0;
        image=test.clone();
        lastImage_test=test;
      }
      if (!lastImage_test.empty())
      {
        image=lastImage_test.clone();
      }
      
      
    } 
    
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void setVIT(arp::VisualInertialTracker& vit){
    vit_ = &vit;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
    /*ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec]",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);*/

    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    
    Eigen::Vector3d acc_S;
    acc_S << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    Eigen::Vector3d omega_S;
    omega_S << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    if(vit_){
      /*if (omega_S.norm()>1)
      {
        ROS_INFO("Imu Seq: [%d]", msg->header.seq);
        ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec]",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      }*/

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
  bool success = false;
  bool ret = true;

  std::cout << std::setprecision(3) << std::fixed;
  std::cout << "" << std::endl;
  std::cout << "Read camera parameters..." << std::endl;
  success = nh.getParam("/arp_node/fu", cp.fu);
  std::cout << "Read parameter fu...    value=" << cp.fu;
  if(success) {
    std::cout << " [ OK ]" << std::endl;
  } else {
    std::cout << " [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/fv", cp.fv);
  std::cout << "Read parameter fv...    value=" << cp.fv;
  if(success) {
    std::cout << " [ OK ]" << std::endl;
  } else {
    std::cout << " [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/cu", cp.cu);
  std::cout << "Read parameter cu...    value=" << cp.cu;
  if(success) {
    std::cout << " [ OK ]" << std::endl;
  } else {
    std::cout << " [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/cv", cp.cv);
  std::cout << "Read parameter cv...    value=" << cp.cv;
  if(success) {
    std::cout << " [ OK ]" << std::endl;
  } else {
    std::cout << " [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/k1", cp.k1);
  std::cout << "Read parameter k1...    value=" << cp.k1;
  if(success) {
    std::cout << "   [ OK ]" << std::endl;
  } else {
    std::cout << "   [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/k2", cp.k2);
  std::cout << "Read parameter k2...    value=" << cp.k2;
  if(success) {
    std::cout << "   [ OK ]" << std::endl;
  } else {
    std::cout << "   [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/p1", cp.p1);
  std::cout << "Read parameter p1...    value=" << cp.p1;
  if(success) {
    std::cout << "   [ OK ]" << std::endl;
  } else {
    std::cout << "   [FAIL]" << std::endl;
    ret = false;
  }
  success = false;
  success = nh.getParam("/arp_node/p2", cp.p2);
  std::cout << "Read parameter p2...    value=" << cp.p2;
  if(success) {
    std::cout << "   [ OK ]" << std::endl;
  } else {
    std::cout << "   [FAIL]" << std::endl;
    ret = false;
  }

  return ret;
}

 /// \brief Read Cam Parameter from launch file and initialize Cam Model
  /// @param[in] nh NodeHandle.
  /// @param[in] cp Struct with camera parameters.
  /// @param[out] camMod Initialized camera model  
arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> setupCamera(ros::NodeHandle& nh, const camParams cp){
  // setup camera model
  const arp::cameras::RadialTangentialDistortion distortion(cp.k1, cp.k2, cp.p1, cp.p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camMod(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv, distortion);
  auto success = camMod.initialiseUndistortMaps(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, cp.fu, cp.fv, cp.cu, cp.cv);
  std::cout << "" << std::endl;
  std::cout << "Setup Camera..." << std::endl;
  std::cout << "Initialize undistort maps...";
  if(success) {
    std::cout << "          [ OK ]" << std::endl;
  } else {
    std::cout << "          [FAIL]" << std::endl;
  }
  std::cout << "" << std::endl;

  return camMod;
}

bool setupVisualInertialTracker(ros::NodeHandle& nh, camParams& cp, arp::VisualInertialTracker& vit){
  //CAM_IMAGE_HEIGHT
  arp::Frontend frontend(CAM_IMAGE_WIDTH,CAM_IMAGE_HEIGHT , cp.fu, cp.fv, cp.cu, cp.cv, cp.k1, cp.k2, cp.p1, cp.p2);

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
  ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
  ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);
  
  // set up EKF
  arp::ViEkf viEkf;
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
  if(readCameraParameters(nh, cp)){
    std::cout << "Camera Parameters read successfully..." << std::endl;
  } else {
    std::cout << "Failed to read some parameters..." << std::endl;
  }
  // setup camera model
  auto phcam = setupCamera(nh, cp);
  // activate camera model
  bool cameraModelApplied = CAM_MODEL; 
  // setup visual inertial tracker
  arp::VisualInertialTracker vit;
  /*if(setupVisualInertialTracker(nh, cp, vit)){
    std::cout << "Setup Visual Inertial Tracker successfully..." << std::endl;
  } else {
    std::cout << "Failed to setup Visual Inertial Tracker..." << std::endl;
  }*/
  arp::Frontend frontend(CAM_IMAGE_WIDTH,CAM_IMAGE_HEIGHT , cp.fu, cp.fv, cp.cu, cp.cv, cp.k1, cp.k2, cp.p1, cp.p2);

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
  ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
  ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);
  
  // set up EKF
  arp::ViEkf viEkf;
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
  vit.setFrontend(frontend);
  vit.setEstimator(viEkf);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  vit.setVisualisationCallback(std::bind(
  &arp::StatePublisher::publish, &pubState, std::placeholders::_1,
  std::placeholders::_2));
  // enable sensor fusion
  vit.enableFusion(ENABLE_FUSION);

  // setup inputs
  Subscriber subscriber;
  subscriber.setVIT(vit);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);
  
  // set up autopilot
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

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
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
      if(subscriber.getLastImage(image)) {
          
          // TODO: add overlays to the cv::Mat image, e.g. text
          cv::Size image_size= image.size();
          // create text to show whether camera model is applied or not
          if(cameraModelApplied){
              cv::Mat tempImage;
              if( !(phcam.undistortImage(image, tempImage)))
              {
                std::cout << "Undistortion failed" << std::endl;
              }
              else
              {
                image=tempImage;
              }
              // resize image to full HD
              cv::resize(image, image,cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_INTER_CUBIC);
              image_size= image.size();
              if(droneStatus==2){
                cv::putText(image, "P: Camera Model (On)", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              } else {
                cv::putText(image, "  Camera Model (On)", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              }
          } else {
              // resize image to full HD
              cv::resize(image, image,cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_INTER_CUBIC);
              image_size= image.size();
              if(droneStatus==2){
                cv::putText(image, "P: Camera Model (Off)", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              } else {
                cv::putText(image, "  Camera Model (Off)", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              }
          }
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
          
          // possible commands in buttom of picture, differentiate: drone is flying or not
          if(droneStatus==3||droneStatus==4||droneStatus==7) {
              cv::putText(image, "W/ S: up/ down", cv::Point(10*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "A/ D: yaw left/ right", cv::Point(10*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "^/ v: for-/ backward", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "</ >: left/ right", cv::Point(image_size.width-370*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
              cv::putText(image, "L: Landing; ESC: Stop", cv::Point(image_size.width/2-185*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);

          }
          else if(droneStatus==2){
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
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    // Press P to toggle application of camera model
    if (droneStatus==2 && state[SDL_SCANCODE_P]) {

      cameraModelApplied = cameraModelApplied ^ 1;
      sleep(1);
      std::cout << "Toggle Camera Model...  status=" << cameraModelApplied << std::endl;
    }

    // TODO: process moving commands when in state 3,4, or 7
    if(droneStatus==3||droneStatus==4||droneStatus==7) {
        double forward=0;
        double left=0;
        double up=0;
        double rotateLeft=0;
        //UP Arrow: move drone forward
        if (state[SDL_SCANCODE_UP]&&!state[SDL_SCANCODE_DOWN]) {
            forward+=1;
            std::cout << "Forward ...     ";

        }
        //Down Arrow: move drone backwards
        if (state[SDL_SCANCODE_DOWN]&&!state[SDL_SCANCODE_UP]) {
            std::cout << "Backwards ...   ";
            forward-=1;
        }
        //Right Arrow: move drone right
        if (state[SDL_SCANCODE_RIGHT]&&!state[SDL_SCANCODE_LEFT]) {
            std::cout << "Right ...       ";
            left-=1;
        }
        //Left Arrow: move drone left
        if (state[SDL_SCANCODE_LEFT]&&!state[SDL_SCANCODE_RIGHT]) {
            std::cout << "Left ...        ";
            left+=1;
        }
        //'W': move drone Up
        if (state[SDL_SCANCODE_W]&&!state[SDL_SCANCODE_S]) {
            std::cout << "Up ...          ";
            up+=1;
        }
        //'S': move drone Down
        if (state[SDL_SCANCODE_S]&&!state[SDL_SCANCODE_W]) {
            std::cout << "Down ...        ";
            up-=1;
        }
        //'A': yaw drone left
        if (state[SDL_SCANCODE_A]&&!state[SDL_SCANCODE_D]) {
            std::cout << "Rotate Left ... ";
            rotateLeft+=1;
        }
        //'D': yaw drone right
        if (state[SDL_SCANCODE_D]&& !state[SDL_SCANCODE_A]) {
            std::cout << "Rotate Right ...";
            rotateLeft-=1;
        }
        if(forward==0&&left==0&&up==0&rotateLeft==0) {
            std::cout << "Hover ...       ";
        }
        std::cout << std::setprecision(2) << std::fixed;
        std::cout << "     status=" << droneStatus;
        std::cout << "...             battery=" << droneBattery;
        //forward moving instructions to Autopilot class
        bool success = autopilot.manualMove(forward, left, up, rotateLeft);
        if (success) {
            std::cout << " [ OK ]" << std::endl;
        } else {
            std::cout << " [FAIL]" << std::endl;
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

