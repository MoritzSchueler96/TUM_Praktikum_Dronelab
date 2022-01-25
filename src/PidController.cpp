/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...
  //compute deltaT
  const double deltaT=(timestampMicroseconds-lastTimestampMicroseconds_)/1000000u;//compute deltaT in seconds

  // compute output:
  double output = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;
  // saturate:
  if (output < minOutput_) {
  output = minOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else if (output > maxOutput_) {
  output = maxOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else {
    //limit deltaT for integration to 0.1s
    if(deltaT<0.1)
    {
      integratedError_ += e * deltaT; // safe to keep integrating
    }
  }
  // save:
  lastTimestampMicroseconds_=timestampMicroseconds;
  return output;

}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
