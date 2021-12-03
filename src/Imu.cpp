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
Eigen::Matrix<float, 15, 15> Imu::calcFc(const RobotState & state, const ImuMeasurement &z)
{
  Eigen::Matrix<float,3,3> Zero= Eigen::MatrixXd::Zero(3, 3);
  Eigen::Matrix<float,3,3> Eye= Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix<float,3,3> cross_v_quat<<       0       , -state.q_WS[3],  state.q_WS[2],
                                          state.q_WS[3] ,     0         , -state.q_WS[1],
                                          -state.q_WS[2],state.q_WS[1]  ,      0        ;
  Eigen::Matrix<float,3,3> R_WS=Eye+2*state.q_WS[4]*cross_v_quat+2*cross_v_quat*cross_v_quat;
  Eigen::Vector3d R_WS_a_s_ba=R_WS*(z.acc_S-state.b_a);
  Eigen::Matrix<float,3,3> cross_R_WS_a_s_ba<<        0       , -R_WS_a_s_ba[3],  R_WS_a_s_ba[2],
                                                R_WS_a_s_ba[3],     0          , -R_WS_a_s_ba[1],
                                               -R_WS_a_s_ba[2],  R_WS_a_s_ba[1],       0        ;
                                    
  Eigen::Matrix<float,15,15> F_C<<  Zero, Zero, Eye, Zero, Zero,
                                    Zero, Zero, Zero, -R_WS, Zero,
                                    Zero, -cross_R_WS_a_s_ba, Zero, Zero, -R_WS,
                                    Zero, Zero, Zero, Zero, Zero,
                                    Zero, Zero, Zero, Zero, Zero;
  return F_C;


}

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
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
  RobotState smalldist=[1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6,1.0e-6 ,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6];
  Eigen::Matrix<double, 15, 15>F_C =calcFc(state_k_minus_1, z_k_minus_1);
  RobotState delta_x1=dt*F_C*smalldist;
  Eigen::Matrix<double, 15, 15>F_C_delta =calcFc(state_k_minus_1+delta_x1, z_k);
  RobotState delta_x2=dt*F_C_delta*smalldist;
  state_k=state_k_minus_1+0.5*(delta_x1+delta_x2);

  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
  jacobian= Eigen::MatrixXd::Identity(15, 15)+0.5*dt*F_C+0.5*dt*F_C_delta*(Eigen::MatrixXd::Identity(15, 15)+F_C);
  }
  return true;
}

}
}  // namespace arp

