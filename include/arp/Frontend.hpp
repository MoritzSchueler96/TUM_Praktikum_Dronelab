/*
 * Frontend.hpp
 *
 *  Created on: 8 Dec 2020
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>

#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/cameras/NoDistortion.hpp>
#include <arp/kinematics/Transformation.hpp>
#include <ros/console.h>

namespace arp {

///\brief This class processes an image and returns the detected marker poses.
class Frontend
{
 public:

  /// \brief Struct holding PID parameters
  struct frontendParams{
    int numKeypoints;
    double mapFocalLength;
    double Brisk_uniformityRadius;
    double Brisk_absoluteThreshold;
    int skipThresInit;
    int skipThresLimit;
    double distanceThres;
  };

  struct ransacParams{
    bool useExtrinsicGuess;
    int iterationsCount;
    float reprojectionError;
    double confidence;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Sets the underlying camera and BRISK parameters (RadialTangentialDistortion)
  /// @param[in] cp Camera Parameter Object
  /// @param[in] numKeypoints The maximum number of Keypoints for BRISK
  /// @param[in] mapFocalLength The focal length of the camera which produced the map
  /// @param[in] Brisk_uniformityRadius Add explanation
  /// @param[in] Brisk_absoluteThreshold Add explanation
  Frontend(arp::cameras::CamParams cp, frontendParams fp);

  /// \brief Load the map
  /// \parameter path The full path to the map file.
  /// \return True on success.
  bool loadMap(std::string path);

  /// \brief Detect and match keypoints in image that can be fed to an estimator.
  /// \warning If not returning true, there may still be detections, but not verified
  ///          and T_CW will be invalid.
  /// \parameter image The input image.
  /// \parameter extractionDirection The extraction direction in camera coordinates.
  ///                                Use (0,1,0) if in vision-only case.
  /// \parameter detections The obtained detections.
  /// \parameter visualisationImage An image to visualise stuff into. Can be the input image.
  /// \parameter needsReInitialisation If true, T_CW *cannot* be used for efficient matching.
  /// \return True on success of RANSAC, i.e. T_CW will be valid.
  bool detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection,
                      DetectionVec & detections, kinematics::Transformation & T_CW, 
                      cv::Mat & visualisationImage, bool needsReInitialisation);

  /// \brief Camera model accessor.
  /// \return Const reference to the underlying camera projection model.
  const arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>& camera() const {
    return camera_;
  }

  /// \brief Enable/disable keypoints
  void showKeypoints(bool enable) {displayKeypoints_ = enable;}
  void showAllKeypoints(bool enable){displayAllKeypoints_=enable;}

  /// Set Ransac Parameters
  void setRansacParams(ransacParams rp) {rp_ = rp;}

 protected:
  /// \brief Detects BRISK (HARRIS) keypoints and extracts descriptors along the extraction direction.
  int detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

  /// \brief Wraps the OpenCV 3D2D pose RANSAC.
  bool ransac(const std::vector<cv::Point3d>& worldPoints, 
              const std::vector<cv::Point2d>& imagePoints, 
              kinematics::Transformation & T_CW, std::vector<int>& inliers) const;

  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camera_; ///< Camera model
  cv::Mat cameraMatrix_; ///< Copy camera model for OpenCV RANSAC
  cv::Mat distCoeffs_; ///< Copy camera model for OpenCV RANSAC

  /// \brief A simple class to store a landmark with position and descriptors.
  struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point; ///< The 3d point in World coordinates.
    std::vector<cv::Mat> descriptors; ///< The descriptors: organised one descriptor per row.
  };
  std::map<uint64_t, Landmark, std::less<uint64_t>, 
      Eigen::aligned_allocator<std::pair<const uint64_t, Landmark> > > landmarks_; ///< Landmarks by ID.

  std::shared_ptr<cv::FeatureDetector> detector_;  ///< the BRISK detector
  std::shared_ptr<cv::DescriptorExtractor> extractor_;  ///< the BRISK extractor

  std::atomic_bool displayKeypoints_{true}; ///< whether to overlay matched keypoints
  std::atomic_bool displayAllKeypoints_{false}; ///< whether to overlay all keypoints
  int skipThresInit_; ///< Reduction Factor to skip landmarks for speed increase
  int skipThresLimit_; ///< Reduction Limit
  double distanceThres_; ///< distance Threshold
  ransacParams rp_; ///< Ransac Parameters

 private:
  Frontend() = delete;
};

}  // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_ */
