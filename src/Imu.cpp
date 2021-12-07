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
Eigen::Matrix<double, 15, 15> calcFc(const RobotState & state, const ImuMeasurement &z)
{
  Eigen::Matrix<double,3,3> Zero=Eigen::Matrix<double, 3,3>::Zero();
  Eigen::Matrix<double,3,3> Eye=Eigen::Matrix<double, 3,3>::Identity();
  /*Eigen::Matrix<double,3,3> cross_v_quat;cross_v_quat<<       0       , -state.q_WS[2],  state.q_WS[1],
                                          state.q_WS[2] ,     0         , -state.q_WS[0],
                                          -state.q_WS[1],state.q_WS[0]  ,      0        ;*/
  Eigen::Matrix<double,3,3> R_WS=state.q_WS.toRotationMatrix();//Eye+2*state.q_WS.w()*cross_v_quat+2*cross_v_quat*cross_v_quat;
  Eigen::Vector3d R_WS_a_s_ba=R_WS*(z.acc_S-state.b_a);
  Eigen::Matrix<double,3,3> cross_R_WS_a_s_ba;cross_R_WS_a_s_ba<<        0       , -R_WS_a_s_ba[3],  R_WS_a_s_ba[2],
                                                R_WS_a_s_ba[3],     0          , -R_WS_a_s_ba[1],
                                               -R_WS_a_s_ba[2],  R_WS_a_s_ba[1],       0        ;
                                    
  Eigen::Matrix<double,15,15> F_C;
  F_C<<  Zero, Zero, Eye, Zero, Zero,
        Zero, Zero, Zero, -R_WS, Zero,
        Zero, -cross_R_WS_a_s_ba, Zero, Zero, -R_WS,
        Zero, Zero, Zero, Zero, Zero,
        Zero, Zero, Zero, Zero, Zero;
  return F_C;


}
RobotState AdditionState_delta(const RobotState & state, const Eigen::Matrix<double,15,1> &delta)
{
  RobotState returnvalue;
  returnvalue.t_WS=state.t_WS+delta.head<3>();
  returnvalue.v_W=state.v_W+delta.segment<3>(6);
  returnvalue.b_g=state.b_g+delta.segment<3>(9);
  returnvalue.b_a=state.b_a+delta.tail<3>();
  Eigen::Quaterniond delta_q_WS;
  double abs=(delta.segment<3>(3).norm()/2);
  delta_q_WS.vec()=sinc(abs)/2*delta.segment<3>(3);
  delta_q_WS.w()=cos(abs);
  returnvalue.q_WS=delta_q_WS*state.q_WS;

  return returnvalue;

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
  Eigen::Matrix<double, 15,1> smalldist;
  smalldist<<1.0e-10, 1.0e-10, 1.0e-10, 1.0e-10, 1.0e-10, 1.0e-10,1.0e-10 ,1.0e-10,1.0e-10,1.0e-10,1.0e-10,1.0e-10,1.0e-10,1.0e-10,1.0e-10;
  
  Eigen::Matrix<double, 15, 15>F_C =calcFc(state_k_minus_1, z_k_minus_1);
  Eigen::Matrix<double, 15,1>delta_x1= dt*F_C*smalldist;
  RobotState state_delta_x1=AdditionState_delta(state_k_minus_1, delta_x1);
  Eigen::Matrix<double, 15, 15>F_C_delta =calcFc(state_delta_x1, z_k);
  Eigen::Matrix<double, 15,1> delta_x2=dt*F_C_delta*smalldist;
  
  state_k=AdditionState_delta(state_k_minus_1, 0.5*(delta_x2+delta_x1));
  
  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
    //std::cout << "F_C_delta:" << std::endl << F_C_delta << std::endl;
    //std::cout << "F_C:" << std::endl << F_C << std::endl;
    auto test=Eigen::Matrix<double, 15,15>::Identity()+0.5*dt*F_C+0.5*dt*F_C_delta*(Eigen::Matrix<double, 15,15>::Identity()+dt*F_C);
    //std::cout << "Test" << std::endl << test << std::endl;
    
    *jacobian=test;
  }
  return true;
}

}
}  // namespace arp

