/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jan 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file implementation/PinholeCamera.hpp
 * @brief Header implementation file for the PinholeCamera class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <stdexcept>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \brief Default Constructor for Distortion
template<class DISTORTION_T>
PinholeCamera<DISTORTION_T>::PinholeCamera(const distortion_t & distortion)
    : PinholeCameraBase(),
    distortion_(distortion),
    fu_(0.0),
    fv_(0.0),
    cu_(0.0),
    cv_(0.0)
{
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
}

/// \brief Constructor for Camera Parameters and Distortion
template<class DISTORTION_T>
PinholeCamera<DISTORTION_T>::PinholeCamera(const arp::cameras::CamParams cp,
                                           const distortion_t & distortion)
    : PinholeCameraBase(cp.imageWidth(), cp.imageHeight()),
    distortion_(distortion),
    fu_(cp.fu()),
    fv_(cp.fv()),
    cu_(cp.cu()),
    cv_(cp.cv())
{
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
}

// Initialise undistort maps to defaults
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::initialiseUndistortMaps() {
  const double f = (fu_ + fv_) * 0.5;
  arp::cameras::CamParams cp(imageWidth(), imageHeight(), f, f, 
      double(imageWidth())*0.5-0.5, double(imageHeight())*0.5-0.5);
  return initialiseUndistortMaps(cp);
}

// Initialise undistort maps, provide custom parameters for the undistorted cam.
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::initialiseUndistortMaps(arp::cameras::CamParams cp) {
			
   // store parameters
   undistortedImageWidth_ = cp.imageWidth();
   undistortedImageHeight_ = cp.imageHeight(); 
   undistortedFocalLengthU_ = cp.fu();
   undistortedFocalLengthV_ = cp.fv();
   undistortedImageCenterU_ = cp.cu();
   undistortedImageCenterV_ = cp.cv();

  // some preparation of the actual and undistorted projections
  Eigen::Matrix2d undistortedK_inv, actualK;
  undistortedK_inv << 1.0/cp.fu(), 0.0, 0.0, 1.0/cp.fv();
  actualK << fu_, 0.0, 0.0, fv_;
  Eigen::Vector2d actualCenter(cu_, cv_);
  Eigen::Vector2d undistortedCenter(cp.cu(), cp.cv());

  // Create the maps and vectors for points
  cv::Mat map_x(cp.imageHeight(), cp.imageWidth(), CV_32F);
  cv::Mat map_y(cp.imageHeight(), cp.imageWidth(), CV_32F);
  Eigen::Vector2d pixel, imgPlane, projectedPoint, distortedPoint, mappedPixel;
  Eigen::Vector3d ray, rayTransformed;
  Eigen::Vector4d hrayTransformed;
  const double rayLength = 0.25;

  for (unsigned int y = 0; y < cp.imageHeight(); y++) {

    pixel(1) = y;

    float *pmap_x = map_x.ptr<float>(y); // for the yth row in the map
    float *pmap_y = map_y.ptr<float>(y);

    for (unsigned int x = 0; x < cp.imageWidth(); x++) {

      pixel(0) = x;

      // Convert from pixels to image plane using ideal camera intrinsics
      imgPlane = undistortedK_inv * (pixel - undistortedCenter);

      // Apply the distortion model to the projection
      distortion_.distort(imgPlane,&distortedPoint);

      // Apply the intrinsics model to get a pixel location
      mappedPixel = (actualK * distortedPoint) + actualCenter;

      // Assign that pixel location to the map
      pmap_x[x] = mappedPixel(0); // assign a value to the (x,y) position in the map
      pmap_y[x] = mappedPixel(1);

    }
  }

  // Apply convertMaps for actual fast remapping later when calling undistortImage
  cv::convertMaps(map_x, map_y, map_x_fast_, map_y_fast_, CV_16SC2);

  return true;
}

// Get the model of the undistorted camera.
template<class DISTORTION_T>
PinholeCamera<NoDistortion> PinholeCamera<DISTORTION_T>::undistortedPinholeCamera() 
    const {
  assert(map_x_fast_.cols !=0);
  arp::cameras::CamParams cp(undistortedImageWidth_, undistortedImageHeight_, 
      undistortedFocalLengthU_, undistortedFocalLengthV_,
      undistortedImageCenterU_, undistortedImageCenterV_);
  return PinholeCamera<NoDistortion>(cp, NoDistortion());
}

// Get undistorted image -- assumes initialiseUndistortMaps was called
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::undistortImage(const cv::Mat & srcImg, 
                                                 cv::Mat & destImg) const {
  assert(map_x_fast_.cols !=0);
  cv::remap(srcImg, destImg, map_x_fast_, map_y_fast_, 
  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
  return true;
}

//////////////////////////////////////////
// Methods to project points

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d* imagePoint) const
{
  // TODO: implement
  // check for null pointers
  if(imagePoint){
      // check if point is behind camera
      if(point[2] <= 0)
      {
        return ProjectionStatus::Behind;
      }
      // check for unstable points
      if(abs(point[2])<1e-12){
        return ProjectionStatus::Invalid;
      }

      // Project Point to unit plane 
      Eigen::Vector2d calc_point, distpoint;
      calc_point << 1/point[2] * point[0], 1/point[2] * point[1];

      // distort point
      if (distortion_.distort(calc_point, &distpoint))
      {
        //Scale Point on image plane
        *imagePoint<<fu_*distpoint[0]+cu_, fv_*distpoint[1]+cv_;

        // check if point lies on the image plane
        if (isInImage(*imagePoint))
        {
          return ProjectionStatus::Successful;
        }
        else
        {
          return ProjectionStatus::OutsideImage;
        }
      }
  }
  return ProjectionStatus::Invalid;
}

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 3> * pointJacobian) const
{
  // TODO: implement in practical No. 3
  Eigen::Vector2d calc_point, distpoint;
  Eigen::Matrix2d U, D;
  Eigen::Matrix<double,2,3> P;
  // check for null pointers
  if(imagePoint && pointJacobian){

      // check if point is behind camera
      if(point[2] <= 0){
        return ProjectionStatus::Behind;
      }

      // check for unstable points
      if(abs(point[2])<1e-12){
        return ProjectionStatus::Invalid;
      }

      //Project Point to unit plane 
      calc_point << 1/point[2] * point[0], 1/point[2] * point[1];

      //calculate Jacobian of projection
      P << 1/point[2], 0, -point[0]/pow(point[2],2),
           0, 1/point[2], -point[1]/pow(point[2],2);

      //distort point and get Distortion Jacobian
      if (distortion_.distort(calc_point, &distpoint, &D)){

          //Scale Point on image plane
          *imagePoint << fu_ * distpoint[0] + cu_, fv_ * distpoint[1] + cv_;
          //calculate Jacobian of scaling
          U << fu_, 0,
               0, fv_;
          //calculate global Jacobian, chain rule
          Eigen::Matrix<double,2,2> temp = U * D;
          *pointJacobian << temp * P;

        // check if point lies on the image plane
          if (isInImage(*imagePoint)){
              return ProjectionStatus::Successful;
          } else {
              return ProjectionStatus::OutsideImage;
          }
      }
  }
return ProjectionStatus::Invalid;
}

/////////////////////////////////////////
// Methods to backproject points

// Back-project a 2d image point into Euclidean space (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction) const
{
  // TODO: implement
  bool success = false;
  // check for null pointers
  if(direction){
      Eigen::Vector2d calc_point, distpoint;
      Eigen::Matrix3d h;
      
      // calculate back projection
      calc_point << one_over_fu_ * (imagePoint[0] - cu_), one_over_fv_ * (imagePoint[1] - cv_);

      // undistort point
      if (distortion_.undistort(calc_point, &distpoint))
      {
        // create 3d direction vector - no unique matching to point possible due to scaling
        *direction << distpoint[0], distpoint[1], 1;    
        success = true;
      }
  }
  return success;

}



}  // namespace cameras
}  // namespace arp
