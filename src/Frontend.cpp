/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

#include <brisk/brisk.h>
#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

namespace arp {

Frontend::Frontend(arp::cameras::CamParams cp, Frontend::frontendParams fp): // numKeypoints=2000, double mapFocalLength=185.6909, double Brisk_uniformityRadius=35, double Brisk_absoluteThreshold=100, int skipThresInit=1, int skipThresLimit=3) :
  camera_(cp, arp::cameras::RadialTangentialDistortion(cp))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = cp.fu();
  cameraMatrix_.at<double>(1,1) = cp.fv();
  cameraMatrix_.at<double>(0,2) = cp.cu();
  cameraMatrix_.at<double>(1,2) = cp.cv();
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = cp.k1();
  distCoeffs_.at<double>(1) = cp.k2();
  distCoeffs_.at<double>(2) = cp.p1();
  distCoeffs_.at<double>(3) = cp.p2();
  skipThresInit_=fp.skipThresInit;
  skipThresLimit_=fp.skipThresLimit;
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(fp.Brisk_uniformityRadius, 0, fp.Brisk_absoluteThreshold, fp.numKeypoints));//10,0,100,2000 Sim: 35, 0, 100, x, Real: 35, 0, 2, 200
  //working limit 25-35 for castle, kitchen 
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
#if 1
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(cp.imageHeight(), cp.imageWidth(), CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(cp.imageHeight(), cp.imageWidth(), CV_32FC(6));
  for (int v=0; v<cp.imageHeight(); ++v) {
    for (int u=0; u<cp.imageWidth(); ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }

  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, fp.mapFocalLength);
#endif   
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  uint64_t id=1;
  while (std::getline(mapfile, line)) {
    Landmark landmark;

    // Convert to stringstream
    std::stringstream ss(line);

    // read 3d position
    for(int i=0; i<3; ++i) {
      std::string coordString;
      std::getline(ss, coordString, ',');
      double coord;
      std::stringstream(coordString) >> coord;
      landmark.point[i] = coord;
    }

    // Get each descriptor
    std::string descriptorstring;
    while(ss.good()){
      std::getline(ss, descriptorstring, ',');
      cv::Mat descriptor(1,48,CV_8UC1);
      for(int col=0; col<48; ++col) {
        uint32_t byte;
        std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
        descriptor.at<uchar>(0,col) = byte;
      }
      landmark.descriptors.push_back(descriptor);
    }

    // store into map
    landmarks_[id] = landmark;
    id++;
  }
  ROS_INFO_STREAM("Loaded " << landmarks_.size() << " landmarks...");
  return landmarks_.size() > 0;
}

int Frontend::detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);
  
  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation& T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size()<5) {
    return false; // not realiable enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, rp_.useExtrinsicGuess, rp_.iterationsCount, rp_.reprojectionError, rp_.confidence, inliers, cv::SOLVEPNP_EPNP);	//false, 300, 5.0, 0.9

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.5);//0.7
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  // print when status changes
  static bool oldIni;
  if(oldIni!=needsReInitialisation)
  {
    ROS_INFO_STREAM("Re-Init: " << needsReInitialisation);
    oldIni=needsReInitialisation;
  }
  
  detections.clear(); // make sure empty

  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);

  // TODO match to map:
  std::vector<cv::Point3d> worldPoints;
  std::vector<cv::Point2d> imagePoints;
  std::vector<uint64_t> lmID;
  int reduceLandmarks = 0;
  static int skipThres = skipThresInit_;
  static int reduceLmCnt = 0;

  ROS_DEBUG_STREAM_THROTTLE(2, "landmarks: " << landmarks_.size());
  ROS_DEBUG_STREAM_THROTTLE(2, "keypoints: " << keypoints.size());

  bool firstrun=true;
  //loop through landmarks
  for(auto & lm : landmarks_) { 

    Eigen::Vector2d landmark2d;
    Eigen::Vector4d landmark4d;
    landmark4d << lm.second.point, 1.0;
    
    // skip invisible landmarks if no Init needed
    // reduce Landmarks by factor 4 if Init needed
    if(!needsReInitialisation){
      skipThres = skipThresInit_;
      reduceLmCnt = 0;
      if(camera_.project((T_CW*landmark4d).head<3>(), &landmark2d)!=arp::cameras::ProjectionStatus::Successful) continue;
    } else {
      reduceLandmarks++;
      reduceLmCnt++;
      if (reduceLmCnt > 750000){
        ROS_DEBUG_STREAM_NAMED("custom", "skip: " << skipThres << " cnt: " << reduceLmCnt);
        reduceLmCnt = 0;
        skipThres = std::min(skipThres+1, skipThresLimit_);
      }
      if(reduceLandmarks > skipThres) reduceLandmarks=0;
       else continue;
    }
    
    // save best matched Points
    bool matched = false;
    float bestDist=60.0; // 60 is given threshold
    Eigen::Vector3d bestLandmarkPt;
    cv::Point2d bestKeyPt;
    int bestLmID;

    for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
      uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
      {
        Eigen::Vector2d keyPt;
        keyPt << keypoints[k].pt.x, keypoints[k].pt.y;
        if(displayKeypoints_&&displayAllKeypoints_&&firstrun) cv::circle(visualisationImage, keypoints[k].pt, 10, cv::Scalar(255,0,0), 1); //blue

        //if pose need no reinitialisation, check if distance of 2D points of landmark and keypoint is below threshold 
        if((!needsReInitialisation) && ((landmark2d - keyPt).norm() > 35.0)) continue;


        for(auto lmDescriptor : lm.second.descriptors) { // check agains all available descriptors
          const float dist = brisk::Hamming::PopcntofXORed(
                  keypointDescriptor, lmDescriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)

          // TODO check if a match and process accordingly
          //check if distance of keypoint is below threshold or distance of already checked keypoints
          if(dist<bestDist)
          {
            //cache actual best match of keypoint to landmark
            bestLandmarkPt=lm.second.point;
            bestKeyPt=keypoints[k].pt;
            bestLmID=lm.first;
            bestDist=dist;
            matched=true;
          }
        }
      }
    }
    if (matched==true)
    {
      //write best matched point to list for ransac 
      worldPoints.push_back(cv::Point3d(bestLandmarkPt(0),bestLandmarkPt(1), bestLandmarkPt(2)) );
      imagePoints.push_back(bestKeyPt);
      lmID.push_back(bestLmID);
      //add marker of matched keypoints to visualisation Image
      if(displayKeypoints_) cv::circle(visualisationImage, bestKeyPt, 10, cv::Scalar(0,0,255), 1); //red
    }
    firstrun=false;
  }
  std::vector<int> inliers;
  // TODO run RANSAC (to remove outliers and get pose T_CW estimate)
  bool returnvalue= ransac(worldPoints, imagePoints, T_CW, inliers);
  ROS_DEBUG_STREAM_THROTTLE(2, "ransac: " << returnvalue);

  // TODO set detections
  //loop through to inlier points of ransac to return list of detections
  for(auto & index: inliers)
  {
    Detection newDetection;
    Eigen::Vector2d keypoint(imagePoints[index].x,imagePoints[index].y);
    Eigen::Vector3d landmark(worldPoints[index].x,worldPoints[index].y,worldPoints[index].z);
    newDetection.keypoint=keypoint;
    newDetection.landmark=landmark;
    newDetection.landmarkId=lmID[index];
    //add inlier points to image 
    if(displayKeypoints_) cv::circle(visualisationImage, imagePoints[index], 10, cv::Scalar(0,255,0), 1); //green
    //add detection
    detections.push_back(newDetection);
  }
  ROS_DEBUG_STREAM_THROTTLE(2, "inliers/total: " << inliers.size()<<" / "<<worldPoints.size());

  // TODO visualise by painting stuff into visualisationImage
  //to safe runtime points are added directly during calculation
  return returnvalue; // TODO return true if successful...
}

}  // namespace arp

