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

Frontend::Frontend(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(10, 0, 100, 2000));
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
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
  std::cout << "loaded " << landmarks_.size() << " landmarks." << std::endl;
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
                      kinematics::Transformation & T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size()<5) {
    return false; // not realiabl enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, false, 100, 5.0, 0.99, inliers, cv::SOLVEPNP_EPNP);	

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

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.7);//0.7
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage)
{
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
  std::vector<int> keypoints_matched;
  std::cout << " keypoints" << keypoints.size()<<"/ "<<landmarks_.size()<<std::endl;
  for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
    bool matched = false;
    uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
    for(auto & lm : landmarks_) { 
      Eigen::Vector2d temp;
      if(camera_.project(lm.second.point,&temp)==arp::cameras::ProjectionStatus::Successful)
      {// go through all landmarks in the map
      for(auto lmDescriptor : lm.second.descriptors) { // check agains all available descriptors
        const float dist = brisk::Hamming::PopcntofXORed(
                keypointDescriptor, lmDescriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
        // TODO check if a match and process accordingly
        if( abs(dist) <60)
        {
          //std::cout << "Distance" <<dist<< std::endl;
        
          matched=true;
          worldPoints.push_back(cv::Point3d(lm.second.point(0),lm.second.point(1), lm.second.point(2)) );
          imagePoints.push_back(keypoints[k].pt);
          lmID.push_back(lm.first);
          break;
        }
      }
      if(matched==true)
      {
        break;
      }
      }
    }
    if (matched==true)
    {
      keypoints_matched.push_back( k);
      cv::circle(visualisationImage, keypoints[k].pt, 2, cv::Scalar(255,0,0), 1);//blue
    
    }
    else
    {
      cv::circle(visualisationImage, keypoints[k].pt, 2, cv::Scalar(128,128,128), 1);//grey
    }
  }
  std::vector<int> inliers;
  // TODO run RANSAC (to remove outliers and get pose T_CW estimate)
  bool returnvalue= ransac(worldPoints, imagePoints, T_CW, inliers);
  
  // TODO set detections
  for(auto & index: inliers)
  {
    Detection newDetection;
    Eigen::Vector2d keypoint(imagePoints[index].x,imagePoints[index].y);
    Eigen::Vector3d landmark(worldPoints[index].x,worldPoints[index].y,worldPoints[index].z);
    newDetection.keypoint=keypoint;
    newDetection.landmark=landmark;
    newDetection.landmarkId=lmID[index];
    cv::circle(visualisationImage, imagePoints[index], 2, cv::Scalar(0,0,255), 1);//red
    //add detection
    detections.push_back(newDetection);
    
  }
  std::cout << " inliers" << inliers.size()<<"/ "<<keypoints_matched.size()<<std::endl;
  // TODO visualise by painting stuff into visualisationImage
  //to safe runtime add points directly during calculation

  /*for(auto k :keypoints_matched)
  {
    auto color=cv::Scalar(255,255,255);
    //check if point is a detection or only a matched landmark
    if(std::find(inliers.begin(),inliers.end(),k)!=inliers.end()) 
    {
      color=cv::Scalar(0,0,255);
    }
    else
    {
      if (std::find(keypoints_matched.begin(),keypoints_matched.end(),k)!=keypoints_matched.end())
      {
        color=cv::Scalar(255,0,0);
      }
    }
    
    cv::circle(visualisationImage, imagePoints[k], 2, color, 1);
    //cv::Size image_size= visualisationImage.size();
    //cv::putText(visualisationImage, "Test", cv::Point(240, 180), cv::FONT_HERSHEY_SIMPLEX,1.5, cv::Scalar(0,255,0), 2, false);
              

  }*/
  

  return returnvalue; // TODO return true if successful...
}

}  // namespace arp

