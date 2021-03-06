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
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>
#include <stdexcept>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0)
{
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(arp::cameras::CamParams cp)
{
  k1_ = cp.k1();
  k2_ = cp.k2();
  p1_ = cp.p1();
  p2_ = cp.p2();
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  // TODO: implement
  bool success = false;
  // check for null pointers
  if(pointDistorted){
      //calculate 
      //calculate squared radius with dot product of point
      double r_squared = pointUndistorted.dot(pointUndistorted);
      //factor of distortion function
      double fact = 1 + k1_ * r_squared + k2_ * pow(r_squared, 2);
      //calculate distorted point with Distortion function from lecture
      *pointDistorted << fact * pointUndistorted[0] + 2*p1_ * pointUndistorted[0] * pointUndistorted[1] + 
              p2_ * (r_squared + 2*pow(pointUndistorted[0], 2)), 
              fact * pointUndistorted[1] + 2*p2_ * pointUndistorted[0] * pointUndistorted[1] + 
              p1_ * (r_squared + 2*pow(pointUndistorted[1], 2));
      success = true;
  }
  //throw std::runtime_error("not implemented");
  return success;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  // TODO: implement
  bool success = false;
  // call distort function to get Distorted Point
  if(RadialTangentialDistortion::distort(pointUndistorted, pointDistorted)){
      // check for null pointer
      if(pointJacobian){
          // get coordinates of point
          double x1 = pointUndistorted[0];
          double x2 = pointUndistorted[1];
          // Calculate intermediate results
          // calculate squared radius with dot product of point
          double r_squared = pointUndistorted.dot(pointUndistorted);
          // factor of distortion function
          double fact = 1 + k1_ * r_squared + k2_ * pow(r_squared, 2);
          // derivation of factor regarding to r_squared
          double dfact = (2*k1_ + 4*k2_ * r_squared);
          // Calculate Jacobian indices after painful derivation by hand
          double J_00 = fact + pow(x1, 2) * dfact + 2*p1_ * x2 + 6*p2_ * x1;
          double J_01 = x1 * x2 * dfact + 2*p1_ * x1 + 2*p2_ * x2;
          double J_11 = fact + pow(x2, 2) * dfact + 2*p2_ * x1 + 6*p1_ * x2;
          
          // write result to Jacobi Matrix
          *pointJacobian << J_00, J_01,
                            J_01, J_11; 
                
          success = true;
      }
  }
  return success;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted; // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  return success;
}

}  // namespace cameras
}  // namespace arp
