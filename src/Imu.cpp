/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */
#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

 /// \brief Calculate Fc from Robot State and IMU measurement
  /// @param[in] state RobotState with current configuration.
  /// @param[in] z latest IMU measurement.
  /// @param[out] Fc The calculated state transition matrix.
Eigen::Matrix<double, 15, 15> calcFc(const RobotState& state, const ImuMeasurement& z)
{
  Eigen::Matrix<double,3,3> Zero=Eigen::Matrix<double, 3,3>::Zero();
  Eigen::Matrix<double,3,3> Eye=Eigen::Matrix<double, 3,3>::Identity();

  Eigen::Matrix<double,3,3> R_WS=state.q_WS.toRotationMatrix();
  Eigen::Vector3d R_WS_a_s_ba=R_WS*(z.acc_S-state.b_a);
  Eigen::Matrix<double,3,3> cross_R_WS_a_s_ba=crossMx(R_WS_a_s_ba);
                                    
  Eigen::Matrix<double,15,15> F_c;
  F_c << Zero, Zero, Eye, Zero, Zero,
         Zero, Zero, Zero, -R_WS, Zero,
         Zero, -cross_R_WS_a_s_ba, Zero, Zero, -R_WS,
         Zero, Zero, Zero, Zero, Zero,
         Zero, Zero, Zero, Zero, Zero;
  return F_c;
}

 /// \brief Calculate fc from Robot State and IMU measurement
  /// @param[in] state RobotState with current configuration.
  /// @param[in] z latest IMU measurement.
  /// @param[out] fc The calculated state derivative.
RobotState calc_fc(const RobotState& state, const ImuMeasurement& z)
{
  RobotState fc;

  Eigen::Quaterniond temp(0,0,0,0);
  temp.vec()=z.omega_S-state.b_g;
  temp.w()=0;

  fc.q_WS=(state.q_WS*temp);
  fc.q_WS.coeffs()*=0.5;
  fc.v_W=state.q_WS.toRotationMatrix()*(z.acc_S-state.b_a)+Eigen::Vector3d(0,0,-9.81);
  fc.t_WS=state.v_W;
  fc.b_a=Eigen::Vector3d::Zero();
  fc.b_g=Eigen::Vector3d::Zero();
  return fc;
}

 /// \brief Adds two robot states.
  /// @param[in] state RobotState with current configuration.
  /// @param[in] delta Second RobotState to be added.
  /// @param[in] normalize Decision if Quaternion needs normalization.
  /// @param[out] outState The calculated state.
RobotState addRobotStates(const RobotState& state, const RobotState& delta, bool normalize)
{
  RobotState outState;
  outState.t_WS=state.t_WS+delta.t_WS;
  outState.v_W=state.v_W+delta.v_W;
  outState.b_g=state.b_g+delta.b_g;
  outState.b_a=state.b_a+delta.b_a;
  outState.q_WS.coeffs()=delta.q_WS.coeffs()+state.q_WS.coeffs();

  if (normalize)
  {
    outState.q_WS.normalize();
  }

  return outState;
}

 /// \brief Multiplies a factor to a robot state.
  /// @param[in] state RobotState with current configuration.
  /// @param[in] factor factor to be multiplied with RobotState.
  /// @param[out] outState The calculated state.
RobotState multRobotState(const RobotState& state, double factor)
{
  RobotState outState;
  outState.t_WS=factor*state.t_WS;
  outState.v_W=factor*state.v_W;
  outState.b_g=factor*state.b_g;
  outState.b_a=factor*state.b_a;
  outState.q_WS.vec()=factor*state.q_WS.vec();
  outState.q_WS.w()=factor*state.q_WS.w();

  return outState;
}

bool Imu::stateTransition(const RobotState& state_k_minus_1,
                          const ImuMeasurement& z_k_minus_1,
                          const ImuMeasurement& z_k, RobotState& state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }
  
  // TODO: implement trapezoidal integration
  //Check if input Quaternion is normalized
  if(abs(state_k_minus_1.q_WS.coeffs().norm()-1)>1e-10)
  {
    ROS_DEBUG("Invalid state_k_minus_1");
    return false;
  }
  RobotState delta_x1= multRobotState(calc_fc(state_k_minus_1,z_k_minus_1),dt);
  RobotState state_delta_x1=addRobotStates(state_k_minus_1, delta_x1, true);

  RobotState delta_x2= multRobotState(calc_fc(state_delta_x1,z_k),dt);
  RobotState delta_x=multRobotState(addRobotStates(delta_x1, delta_x2, false),0.5);
  state_k=addRobotStates(state_k_minus_1, delta_x, true);
  
  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
    Eigen::Matrix<double, 15, 15> F_c =calcFc(state_k_minus_1, z_k_minus_1);
    Eigen::Matrix<double, 15, 15>F_c_delta =calcFc(state_delta_x1, z_k);
    
    *jacobian=Eigen::Matrix<double, 15,15>::Identity()+0.5*dt*F_c+0.5*dt*F_c_delta*(Eigen::Matrix<double, 15,15>::Identity()+dt*F_c);
  
    return true;
  }
  return false;
}

}
}  // namespace arp

