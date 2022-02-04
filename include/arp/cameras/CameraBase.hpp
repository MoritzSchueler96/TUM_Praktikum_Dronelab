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
 *********************************************************************************/

/**
 * @file cameras/CameraBase.hpp
 * @brief Header file for the CameraBase class.
 * @author Stefan Leutenegger
 */


#ifndef INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_
#define INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_

#include <vector>
#include <memory>
#include <stdint.h>
#include <Eigen/Core>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <arp/cameras/DistortionBase.hpp>
#include <ros/console.h>

/// \brief arp Main namespace of this package.
namespace arp {

/// \brief A simple struct containing all the necessary information about a
///        keypoint detection.
struct Detection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d keypoint; ///< The 2d keypoint detection.
  Eigen::Vector3d landmark; ///< The 3d landmark in World coordinates.
  uint64_t landmarkId; ///< The corresponding landmark ID.
};
typedef std::vector<Detection, Eigen::aligned_allocator<Detection>> DetectionVec;

/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class CamParams
/// \brief Base class for all camera parameters.
class CamParams
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief default Constructor -- does nothing serious
  inline CamParams()
      : imageWidth_(0),
        imageHeight_(0),
        fu_(0.0),
        fv_(0.0),
        cu_(0.0),
        cv_(0.0),
        k1_(0.0),
        k2_(0.0),
        p1_(0.0),
        p2_(0.0)
  {
  }

  /// \brief Constructor for width, height
  inline CamParams(int imageWidth, int imageHeight)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight)
    {
    }

  /// \brief Constructor for width, height, fu, fv, cu and cv
  inline CamParams(int imageWidth, int imageHeight, double fu, double fv, double cu, double cv)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight),
          fu_(fu),
          fv_(fv),
          cu_(cu),
          cv_(cv)
    {
    }

  /// \brief Constructor for width, height, fu, fv, cu, cv, k1, k2, p1 and p2
  inline CamParams(int imageWidth, int imageHeight, double fu, double fv, double cu, double cv, double k1, double k2, double p1, double p2)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight),
          fu_(fu),
          fv_(fv),
          cu_(cu),
          cv_(cv),
          k1_(k1),
          k2_(k2),
          p1_(p1),
          p2_(p2)
    {
    }

  /// \brief Constructor for cameraParams
  inline CamParams(const arp::cameras::CamParams& cp)
        : imageWidth_(cp.imageWidth()),
          imageHeight_(cp.imageHeight()),
          fu_(cp.fu()),
          fv_(cp.fv()),
          cu_(cp.cu()),
          cv_(cp.cv()),
          k1_(cp.k1()),
          k2_(cp.k2()),
          p1_(cp.p1()),
          p2_(cp.p2())
    {
    }

  /// \brief Constructor for k1, k2, p1 and p2
  inline CamParams(double k1, double k2, double p1, double p2)
        : k1_(k1),
          k2_(k2),
          p1_(p1),
          p2_(p2)
    {
    }

  /// \brief Destructor -- does nothing
  inline virtual ~CamParams()
  {
  }

   /// \brief The width of the image in pixels.
  inline uint32_t imageWidth() const
  {
    return imageWidth_;
  }
  /// \brief The height of the image in pixels.
  inline uint32_t imageHeight() const
  {
    return imageHeight_;
  }

  /// \brief The horizontal focal length in pixels.
  inline double fu() const
  {
    return fu_;
  }

  /// \brief The vertical focal length in pixels.
  inline double fv() const
  {
    return fv_;
  }

  /// \brief The horizontal centre in pixels.
  inline double cu() const
  {
    return cu_;
  }

  /// \brief The vertical centre in pixels.
  inline double cv() const
  {
    return cv_;
  }

  /// \brief The radial parameter 1.
  inline double k1() const
  {
    return k1_;
  }

  /// \brief The radial parameter 2.
  inline double k2() const
  {
    return k2_;
  }

  /// \brief The tangential parameter 1.
  inline double p1() const
  {
    return p1_;
  }

  /// \brief The tangential parameter 2.
  inline double p2() const
  {
    return p2_;
  }

  /// \brief setter to set all parameters at once
  inline bool setParams(int imageWidth, int imageHeight, double fu, double fv, double cu, double cv, double k1, double k2, double p1, double p2)
  {
      imageWidth_ = imageWidth;
      imageHeight_ = imageHeight;
      fu_ = fu;
      fv_ = fv;
      cu_ = cu;
      cv_ = cv;
      k1_ = k1;
      k2_ = k2;
      p1_ = p1;
      p2_ = p2;

      return true;
  }

 protected:

  int imageWidth_;  ///< image width in pixels
  int imageHeight_;  ///< image height in pixels
  double fu_;  ///< horizontal focal length in pixels
  double fv_;  ///< vertical focal length in pixels
  double cu_;  ///< horizontal centre in pixels
  double cv_;  ///< vertical centre in pixels
  double k1_;  ///< radial parameter 1
  double k2_;  ///< radial parameter 2
  double p1_;  ///< tangential parameter 1
  double p2_;  ///< tangential parameter 2

};

/// \class ProjectionStatus
/// \brief Indicates what happened when applying any of the project functions.
enum class ProjectionStatus
{
  Successful,
  OutsideImage,
  Behind,
  Invalid
};

/// \class CameraBase
/// \brief Base class for all camera models.
class CameraBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief default Constructor -- does nothing serious
  inline CameraBase()
      : imageWidth_(0),
        imageHeight_(0)
  {
  }

  /// \brief Constructor for width, height and Id
  inline CameraBase(int imageWidth, int imageHeight)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight)
    {
    }

  /// \brief Destructor -- does nothing
  inline virtual ~CameraBase()
  {
  }

  /// \brief The width of the image in pixels.
  inline uint32_t imageWidth() const
  {
    return imageWidth_;
  }
  /// \brief The height of the image in pixels.
  inline uint32_t imageHeight() const
  {
    return imageHeight_;
  }

  //////////////////////////////////////////////////////////////
  /// \name Methods to project points
  /// @{

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Euclidean coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(const Eigen::Vector3d & point,
                                   Eigen::Vector2d * imagePoint) const = 0;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(
      const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
      Eigen::Matrix<double, 2, 3> * pointJacobian) const = 0;
			
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to backproject points
  /// @{

  /// \brief Back-project a 2d image point into Euclidean space (direction vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The Euclidean direction vector.
  /// @return     true on success.
  virtual bool backProject(const Eigen::Vector2d & imagePoint,
                           Eigen::Vector3d * direction) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to facilitate unit testing
  /// @{

  /// \brief Creates a random (uniform distribution) image point.
  /// @return A random image point.
  virtual Eigen::Vector2d createRandomImagePoint() const;

  /// \brief Creates a random visible point in Euclidean coordinates.
  /// @param[in] minDist The minimal distance of this point.
  /// @param[in] maxDist The maximum distance of this point.
  /// @return    A random Euclidean point.
  virtual Eigen::Vector3d createRandomVisiblePoint(double minDist = 0.0,
                                                   double maxDist = 10.0) const;
  /// @}

 protected:

  /// \brief Check if the keypoint is in the image.
  inline bool isInImage(const Eigen::Vector2d& imagePoint) const;

  int imageWidth_;  ///< image width in pixels
  int imageHeight_;  ///< image height in pixels
};

}  // namespace cameras
}  // namespace drltools

#include "implementation/CameraBase.hpp"

#endif /* INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_ */
