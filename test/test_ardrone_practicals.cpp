
#include <gtest/gtest.h>
// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
// Bring in gtest


#include <iostream>

// Test the projection and unprojection
TEST(PinholeCamera, projectBackProject)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // project
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint);
  // backProject
  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);
  
  // now they should align:
  EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);

  
}
TEST(PinholeCamera, projectBackProject_wo_Distortion)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::NoDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // project
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint);
  // backProject
  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);
  
  // now they should align:
  EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);

  
}
TEST(PinholeCamera, OutofImage)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  point_D[0]=1000;
  Eigen::Vector2d imagePoint;
  // project
  auto test= pinholeCamera.project(point_D,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::OutsideImage);
}
TEST(PinholeCamera, InsideImage)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  Eigen::Vector2d imagePoint;
  // project
  auto test= pinholeCamera.project(point_D,&imagePoint);
  // now they should align:
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Successful);
}
TEST(PinholeCamera, ZeroDepth)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  point_D[2]=0;
  Eigen::Vector2d imagePoint;
  // project
  auto test= pinholeCamera.project(point_D,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Behind);
}
TEST(PinholeCamera, BehindCamera)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  point_D[2]=-10;
  Eigen::Vector2d imagePoint;
  // project
  auto test= pinholeCamera.project(point_D,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Behind);
}
TEST(PinholeCamera, InvalidImagePointProject)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  // Eigen::Vector3d *point_D = NULL;
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  Eigen::Vector2d *imagePoint = NULL;
  // project
  auto test= pinholeCamera.project(point_D, imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Invalid);
}
TEST(PinholeCamera, InvalidDirectionBackProject)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  Eigen::Vector2d imagePoint;
  Eigen::Vector3d *direction = NULL;
  // back project
  auto test= pinholeCamera.backProject(imagePoint, direction);
  EXPECT_TRUE(test==false);
}
TEST(PinholeCamera, InvalidJacobian)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> *pointJacobian = NULL;
  // project
  auto test= pinholeCamera.project(point_D,&imagePoint, pointJacobian);
  // now they should align:
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Invalid);
}
TEST(PinholeCamera, Jacobian_wo_Dist)
{
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::NoDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();

  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;
  // project
  pinholeCamera.project(point_D,&imagePoint, &pointJacobian);
  Eigen::Matrix<double, 2, 3> numerical_pointJacobian;
  double delta=1e-4;
  for(int i=0; i<3;i++)
  {
    auto point_D_pos=point_D;
    auto point_D_neg=point_D;
    point_D_pos[i]+=delta;
    point_D_neg[i]-=delta;
    Eigen::Vector2d imagePoint_pos;
    Eigen::Vector2d imagePoint_neg;
    pinholeCamera.project(point_D_pos,&imagePoint_pos);
    pinholeCamera.project(point_D_neg,&imagePoint_neg);
    numerical_pointJacobian(0,i)=(imagePoint_pos[0]-imagePoint_neg[0])/(2*delta);
    numerical_pointJacobian(1,i)=(imagePoint_pos[1]-imagePoint_neg[1])/(2*delta);

  }
   
  //std::cout<<"Jacobian:\n"<<pointJacobian<<"\n";
  //std::cout<<"Numerical Jacobian:\n"<<numerical_pointJacobian<<"\n";
  // now they should align:
  EXPECT_TRUE((pointJacobian-numerical_pointJacobian).cwiseAbs().sum()<1.0e-3);
}
TEST(PinholeCamera, Jacobian_w_Dist)
{
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();
  auto point_D = pinholeCamera.createRandomVisiblePoint();

  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;
  // project
  pinholeCamera.project(point_D,&imagePoint, &pointJacobian);
  Eigen::Matrix<double, 2, 3> numerical_pointJacobian;
  double delta=1e-4;
  for(int i=0; i<3;i++)
  {
    auto point_D_pos=point_D;
    auto point_D_neg=point_D;
    point_D_pos[i]+=delta;
    point_D_neg[i]-=delta;
    Eigen::Vector2d imagePoint_pos;
    Eigen::Vector2d imagePoint_neg;
    pinholeCamera.project(point_D_pos,&imagePoint_pos);
    pinholeCamera.project(point_D_neg,&imagePoint_neg);
    numerical_pointJacobian(0,i)=(imagePoint_pos[0]-imagePoint_neg[0])/(2*delta);
    numerical_pointJacobian(1,i)=(imagePoint_pos[1]-imagePoint_neg[1])/(2*delta);

  }
   
  //std::cout<<"Jacobian:\n"<<pointJacobian<<"\n";
  //std::cout<<"Numerical Jacobian:\n"<<numerical_pointJacobian<<"\n";
  // now they should align:
  EXPECT_TRUE((pointJacobian-numerical_pointJacobian).cwiseAbs().sum()<1.0e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

