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
#include <opencv2/core/eigen.hpp>
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
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(10, 0, 100, 500));//10,0,100,2000
  //working limit 25-35 for castle, kitchen 
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
  if(logLevel >= logRELEASE) std::cout << "Loaded " << landmarks_.size() << " landmarks..." << std::endl;
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

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.5);//0.7
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
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
  std::vector<Landmark> landmarks;
  std::vector<uint64_t> lms;
  int reduceLandmarks=0;

  if(logLevel == logDEBUG1) std::cout << " landmarks" << landmarks_.size()<<"/ "<<lms.size()<<std::endl;
  if(logLevel == logDEBUG1) std::cout << " init" << needsReInitialisation<<std::endl;
  if(logLevel == logDEBUG1) std::cout << " keypoints" << keypoints.size()<<std::endl;

  //loop through landmarks
  for(auto & lm : landmarks_) { 
    Eigen::Vector2d temp;
    Eigen::Vector4d temp4d;

    temp4d << lm.second.point, 1.0;

    std::cout << "T_CW: " << T_CW.T() << "lm.sc.pt: " << lm.second.point << "temp4d: " << temp4d << std::endl;
    // skip invisible landmarks
    /*
    if (!needsReInitialisation && !(camera_.project(T_CW*lm.second.point, &temp)==arp::cameras::ProjectionStatus::Successful)){
      continue;
    }*/
    camera_.project(T_CW*lm.second.point, &temp);
    
    bool matched = false;
    float bestDist=60.0;//60 is given threshold
    Eigen::Vector3d tempPoint3d;
    cv::Point2d tempPoint2d;
    int lmID_temp;
    int keyID_temp;

    for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
      uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
      {
        Eigen::Vector2d t2d;
        std::cout << "key: " << keypoints[k].pt << "temp: " << temp << std::endl; 
        cv2eigen(keypoints[k].pt, t2d);
        std::cout << "key: " << t2d << "temp: " << temp << std::endl; 
        std::cout << "diff: " << temp - t2d << "norm: " << (temp - t2d).norm() << std::endl;
        if((temp - t2d).norm() > 30.0){
            std::cout << "bad" << std::endl;
            continue;
        }
        for(auto lmDescriptor : lm.second.descriptors) { // check agains all available descriptors
          const float dist = brisk::Hamming::PopcntofXORed(
                  keypointDescriptor, lmDescriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
          // TODO check if a match and process accordingly
          //check if distance of landmark is below threshold or distance of already checked landmarks
          if(dist<bestDist)
          {
            //cache actual best match
            tempPoint3d=lm.second.point;
            tempPoint2d=keypoints[k].pt;
            lmID_temp=lm.first;
            keyID_temp = k;
            bestDist=dist;
            matched=true;
          }
        }
      }
    }
    if (matched==true)
    {
      //write best matched point to list for ransac 
      worldPoints.push_back(cv::Point3d(tempPoint3d(0),tempPoint3d(1), tempPoint3d(2)) );
      imagePoints.push_back(tempPoint2d);
      lmID.push_back(lmID_temp);
      keypoints_matched.push_back(keyID_temp);
      //add marker of matched keypoints to visualisation Image
      if(displayKeypoints_) cv::circle(visualisationImage, tempPoint2d, 10, cv::Scalar(0,0,255), 1); //red
    }
  }
  std::vector<int> inliers;
  // TODO run RANSAC (to remove outliers and get pose T_CW estimate)
  bool returnvalue= ransac(worldPoints, imagePoints, T_CW, inliers);
  if(logLevel == logDEBUG1) std::cout << " ransac" << returnvalue<<std::endl;

  // TODO set detections
  //loop through to inlier points of ransac to return detections
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
  if(logLevel == logDEBUG1) std::cout << " inliers" << inliers.size()<<"/ "<<worldPoints.size()<<std::endl;

  // TODO visualise by painting stuff into visualisationImage
  //to safe runtime add points directly during calculation
  return returnvalue; // TODO return true if successful...
}

}  // namespace arp

