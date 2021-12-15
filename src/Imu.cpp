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
  Eigen::Matrix<double,3,3> cross_R_WS_a_s_ba=crossMx(R_WS_a_s_ba);
  /*cross_R_WS_a_s_ba<<        0       , -R_WS_a_s_ba[3],  R_WS_a_s_ba[2],
                                                R_WS_a_s_ba[3],     0          , -R_WS_a_s_ba[1],
                                               -R_WS_a_s_ba[2],  R_WS_a_s_ba[1],       0        ;*/
                                    
  Eigen::Matrix<double,15,15> F_C;
  F_C<<  Zero, Zero, Eye, Zero, Zero,
        Zero, Zero, Zero, -R_WS, Zero,
        Zero, -cross_R_WS_a_s_ba, Zero, Zero, -R_WS,
        Zero, Zero, Zero, Zero, Zero,
        Zero, Zero, Zero, Zero, Zero;
  return F_C;


}
RobotState calc_fc(const RobotState & state, const ImuMeasurement &z)
{
  RobotState returnvalue;
  returnvalue.t_WS=state.v_W;
  Eigen::Quaterniond temp(0,0,0,0);
  temp.vec()=z.omega_S-state.b_g;
  temp.w()=0;
  returnvalue.q_WS=(state.q_WS*temp);
  returnvalue.q_WS.coeffs()*=0.5;
  //returnvalue.q_WS.w()=0.5*state.q_WS.coeffs();
  //returnvalue.q_WS.normalize();
  returnvalue.v_W=state.q_WS.toRotationMatrix()*(z.acc_S-state.b_a)+Eigen::Vector3d(0,0,-9.81);
  returnvalue.b_a=Eigen::Vector3d::Zero();
  returnvalue.b_g=Eigen::Vector3d::Zero();
  //std::cout << "calc_fc return:" << std::endl << returnvalue.v_W << std::endl;
  return returnvalue;

}
RobotState AdditionState_delta(const RobotState & state, const RobotState &delta, bool normalize)
{
  RobotState returnvalue;
  returnvalue.t_WS=state.t_WS+delta.t_WS;
  returnvalue.v_W=state.v_W+delta.v_W;
  returnvalue.b_g=state.b_g+delta.b_g;
  returnvalue.b_a=state.b_a+delta.b_a;
  returnvalue.q_WS.coeffs()=delta.q_WS.coeffs()+state.q_WS.coeffs();
  //returnvalue.q_WS.w()=delta.q_WS.w()+state.q_WS.w();
  if (normalize)
  {
    returnvalue.q_WS.normalize();
  }

  return returnvalue;

}
RobotState MultiplicationState(const RobotState & state, double factor)
{
  RobotState returnvalue;
  returnvalue.t_WS=factor*state.t_WS;
  returnvalue.v_W=factor*state.v_W;
  returnvalue.b_g=factor*state.b_g;
  returnvalue.b_a=factor*state.b_a;
  returnvalue.q_WS.vec()=factor*state.q_WS.vec();
  returnvalue.q_WS.w()=factor*state.q_WS.w();
  //returnvalue.q_WS.normalize();

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
 
  RobotState delta_x1= MultiplicationState(calc_fc(state_k_minus_1,z_k_minus_1),dt);
  RobotState state_delta_x1=AdditionState_delta(state_k_minus_1, delta_x1, true);

  
  RobotState delta_x2= MultiplicationState(calc_fc(state_delta_x1,z_k),dt);
  RobotState delta_x=MultiplicationState(AdditionState_delta(delta_x1, delta_x2, false),0.5);
  state_k=AdditionState_delta(state_k_minus_1, delta_x, true);
  //state_k.q_WS.normalize();
  
  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
    Eigen::Matrix<double, 15, 15> F_C =calcFc(state_k_minus_1, z_k_minus_1);
    Eigen::Matrix<double, 15, 15>F_C_delta =calcFc(state_delta_x1, z_k);
    
    *jacobian=Eigen::Matrix<double, 15,15>::Identity()+0.5*dt*F_C+0.5*dt*F_C_delta*(Eigen::Matrix<double, 15,15>::Identity()+dt*F_C);
  
    return true;
  }
  return false;
}

}
}  // namespace arp

