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

// \brief arp Main namespace of this package.
namespace arp {
// \brief cameras Namespace for camera-related functionality.
namespace cameras {

template<class DISTORTION_T>
PinholeCamera<DISTORTION_T>::PinholeCamera(int imageWidth,
                                           int imageHeight,
                                           double focalLengthU,
                                           double focalLengthV,
                                           double imageCenterU,
                                           double imageCenterV,
                                           const distortion_t & distortion)
    : PinholeCameraBase(imageWidth, imageHeight),
    distortion_(distortion),
    fu_(focalLengthU),
    fv_(focalLengthV),
    cu_(imageCenterU),
    cv_(imageCenterV)
{
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
}

// Initialise undistort maps to defaults
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::initialiseUndistortMaps() {
  const double f = (fu_ + fv_) * 0.5;
  return initialiseUndistortMaps(imageWidth(), imageHeight(), f, f, 
      double(imageWidth())*0.5-0.5, double(imageHeight())*0.5-0.5);
}

// Initialise undistort maps, provide custom parameters for the undistorted cam.
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::initialiseUndistortMaps(
    int undistortedImageWidth, int undistortedImageHeight, 
    double undistortedFocalLengthU, double undistortedFocalLengthV, 
    double undistortedImageCenterU, double undistortedImageCenterV) {
			
   // store parameters
   undistortedImageWidth_ = undistortedImageWidth;
   undistortedImageHeight_ = undistortedImageHeight; 
   undistortedFocalLengthU_ = undistortedFocalLengthU;
   undistortedFocalLengthV_ = undistortedFocalLengthV;
   undistortedImageCenterU_ = undistortedImageCenterU;
   undistortedImageCenterV_ = undistortedImageCenterV;

  // some preparation of the actual and undistorted projections
  Eigen::Matrix2d undistortedK_inv, actualK;
  undistortedK_inv << 1.0/undistortedFocalLengthU, 0.0, 0.0, 1.0/undistortedFocalLengthV;
  actualK << fu_, 0.0, 0.0, fv_;
  Eigen::Vector2d actualCenter(cu_, cv_);
  Eigen::Vector2d undistortedCenter(undistortedImageCenterU, undistortedImageCenterV);

  // Create the maps and vectors for points
  cv::Mat map_x(undistortedImageHeight, undistortedImageWidth, CV_32F);
  cv::Mat map_y(undistortedImageHeight, undistortedImageWidth, CV_32F);
  Eigen::Vector2d pixel, imgPlane, projectedPoint, distortedPoint, mappedPixel;
  Eigen::Vector3d ray, rayTransformed;
  Eigen::Vector4d hrayTransformed;
  const double rayLength = 0.25;

  for (unsigned int y = 0; y < undistortedImageHeight; y++) {

    pixel(1) = y;

    float *pmap_x = map_x.ptr<float>(y); // for the yth row in the map
    float *pmap_y = map_y.ptr<float>(y);

    for (unsigned int x = 0; x < undistortedImageWidth; x++) {

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
  return PinholeCamera<NoDistortion>(undistortedImageWidth_, undistortedImageHeight_, 
      undistortedFocalLengthU_, undistortedFocalLengthV_,
      undistortedImageCenterU_, undistortedImageCenterV_, 
      NoDistortion());
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
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint) const
{
  // TODO: implement
 if(point[2]==0)
  {
    return ProjectionStatus::Invalid;
  }
  Eigen::Vector2d calc_point, distpoint;
  calc_point<<1/point[2]*point[0], 1/point[2]*point[1];
  if (distortion_.distort(calc_point, &distpoint))
  {
    *imagePoint<<fu_*distpoint[0]+cu_, fv_*distpoint[1]+cv_;
    //std::cout<<"fu"<<fu_<<"cu"<<cu_<<"fv"<<fv_<<"cv"<<cv_<<"\n";

    if (isInImage(*imagePoint))
    {
      return ProjectionStatus::Successful;
    }
    else
    {
      return ProjectionStatus::OutsideImage;
    }
  }
  
  //throw std::runtime_error("not implemented");
  return ProjectionStatus::Invalid;
}

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 3> * pointJacobian) const
{
  // TODO: implement in practical No. 3
  //throw std::runtime_error("not implemented");

  Eigen::Vector2d calc_point, distpoint;
  Eigen::Matrix<double,2,2> U, D;
  Eigen::Matrix<double,2,3> P;
  if(point[2]==0)
  {
    return ProjectionStatus::Invalid;
  }
  //Project Point to unit plane 
  calc_point<<1/point[2]*point[0], 1/point[2]*point[1];
  //calculate Jacobian of projection
  P<<1/point[2], 0, -point[0]/pow(point[2],2),
  0,1/point[2], -point[1]/pow(point[2],2);
  //distort point and get Disytortion Jacobian
  if (distortion_.distort(calc_point, &distpoint, &D))
  {
    //Scale Point on image plane
    *imagePoint<<fu_*int(distpoint[0])+cu_, fv_*int(distpoint[1])+cv_;
    //calculate Jacobian of scaling
    U<<fu_, 0,
        0, fv_;
    //calculate global Jacobian, chain rule
    Eigen::Matrix<double,2,2> temp=U*D;

    *pointJacobian<<temp*P;
    if (isInImage(*imagePoint))
    {
      return ProjectionStatus::Successful;
    }
    else
    {
      return ProjectionStatus::OutsideImage;
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
  Eigen::Vector2d calc_point, distpoint;
  Eigen::Matrix3d h;
  
  calc_point<< one_over_fu_*(imagePoint[0]-cu_), one_over_fv_*(imagePoint[1]-cv_);
  if (distortion_.undistort(calc_point, &distpoint))
  {
    *direction<<distpoint[0],distpoint[1],1;    
    return true;

  }
  //throw std::runtime_error("not implemented");
  return false;
}



}  // namespace cameras
}  // namespace arp
