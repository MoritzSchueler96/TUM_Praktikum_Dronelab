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
#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 960
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

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
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
          
          // resize image
          cv::resize(image, image,cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_INTER_CUBIC);

          // TODO: add overlays to the cv::Mat image, e.g. text
          //generate Text for drone state and add it to picture
          cv::Size image_size= image.size();
          std::string display="state: "+std::to_string(droneStatus);
          cv::putText(image, display, cv::Point(10*FONT_SCALING, 50*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING*2, FONT_COLOR, 2, false); //putText( image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)
          //creates text of Batterie charge in 0.1% and add it to picture
          std::stringstream stream;
          stream<< std::fixed<<std::setprecision(1)<<droneBattery <<"%";
          auto color = FONT_COLOR;
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
              cv::putText(image, "T: Taking off; ESC: Stop", cv::Point(image_size.width/2-200*FONT_SCALING, image_size.height-10*FONT_SCALING), cv::FONT_HERSHEY_SIMPLEX,FONT_SCALING, FONT_COLOR, 2, false);
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

    // TODO: process moving commands when in state 3,4, or 7
    if(droneStatus==3||droneStatus==4||droneStatus==7) {
        double forward=0;
        double left=0;
        double up=0;
        double rotateLeft=0;
        //UP Arrow: move drone forward
        if (state[SDL_SCANCODE_UP]&&!state[SDL_SCANCODE_DOWN]) {
            forward+=1;
            std::cout << "Forward ";

        }
        //Down Arrow: move drone backwards
        if (state[SDL_SCANCODE_DOWN]&&!state[SDL_SCANCODE_UP]) {
            std::cout << "Backwards ";
            forward-=1;
        }
        //Right Arrow: move drone right
        if (state[SDL_SCANCODE_RIGHT]&&!state[SDL_SCANCODE_LEFT]) {
            std::cout << "Right ";
            left-=1;
        }
        //Left Arrow: move drone left
        if (state[SDL_SCANCODE_LEFT]&&!state[SDL_SCANCODE_RIGHT]) {
            std::cout << "Left ";
            left+=1;
        }
        //'W': move drone Up
        if (state[SDL_SCANCODE_W]&&!state[SDL_SCANCODE_S]) {
            std::cout << "Up ";
            up+=1;
        }
        //'S': move drone Down
        if (state[SDL_SCANCODE_S]&&!state[SDL_SCANCODE_W]) {
            std::cout << "Down ";
            up-=1;
        }
        //'A': yaw drone left
        if (state[SDL_SCANCODE_A]&&!state[SDL_SCANCODE_D]) {
            std::cout << "Rotate Left ";
            rotateLeft+=1;
        }
        //'D': yaw drone right
        if (state[SDL_SCANCODE_D]&& !state[SDL_SCANCODE_A]) {
            std::cout << "Rotate Right ";
            rotateLeft-=1;
        }
        if(forward==0&&left==0&&up==0&rotateLeft==0) {
            std::cout << "Hover ";
        }
        std::cout << "...  status=" << droneStatus;
        std::cout << "...  battery=" << droneBattery;
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

