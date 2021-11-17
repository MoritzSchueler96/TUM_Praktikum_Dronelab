
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
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
  // set x coordinate to outside of the image plane
  point_C[0]=1000;
  Eigen::Vector2d imagePoint;

  // project
  auto test= pinholeCamera.project(point_C,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::OutsideImage);
}
TEST(PinholeCamera, InsideImage)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
  Eigen::Vector2d imagePoint;
  // project
  auto test= pinholeCamera.project(point_C,&imagePoint);
  // now they should align:
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Successful);
}
TEST(PinholeCamera, ZeroDepth)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
  // set z coordinate to lie on the image plane
  point_C[2]=0;
  Eigen::Vector2d imagePoint;

  // project
  auto test= pinholeCamera.project(point_C,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Behind);
}
TEST(PinholeCamera, BehindCamera)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();
  // set z coordinate to lie behind the camera
  point_C[2]=-10;
  Eigen::Vector2d imagePoint;

  // project
  auto test= pinholeCamera.project(point_C,&imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Behind);
}
TEST(PinholeCamera, InvalidImagePointProject)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // create null pointer
  Eigen::Vector2d *imagePoint = NULL;
  // project
  auto test= pinholeCamera.project(point_C, imagePoint);
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Invalid);
}
TEST(PinholeCamera, InvalidDirectionBackProject)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create an arbitrary image point
  Eigen::Vector2d imagePoint;
  // create null pointer
  Eigen::Vector3d *direction = NULL;

  // back project
  auto test= pinholeCamera.backProject(imagePoint, direction);
  EXPECT_TRUE(test==false);
}
TEST(PinholeCamera, InvalidJacobian)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // create an arbitrary image point
  Eigen::Vector2d imagePoint;
  // create null pointer
  Eigen::Matrix<double, 2, 3> *pointJacobian = NULL;

  // project
  auto test= pinholeCamera.project(point_C,&imagePoint, pointJacobian);
  // now they should align:
  EXPECT_TRUE(test==arp::cameras::ProjectionStatus::Invalid);
}
TEST(PinholeCamera, Jacobian_wo_Dist)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::NoDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // create image point and jacobian
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;

  // project
  pinholeCamera.project(point_C,&imagePoint, &pointJacobian);

  // calculate jacobian via numeric differences
  Eigen::Matrix<double, 2, 3> numerical_pointJacobian;
  double delta=1e-4;
  for(int i=0; i<3;i++)
  {
    auto point_C_pos=point_C;
    auto point_C_neg=point_C;
    point_C_pos[i]+=delta;
    point_C_neg[i]-=delta;
    Eigen::Vector2d imagePoint_pos;
    Eigen::Vector2d imagePoint_neg;
    pinholeCamera.project(point_C_pos,&imagePoint_pos);
    pinholeCamera.project(point_C_neg,&imagePoint_neg);
    numerical_pointJacobian(0,i)=(imagePoint_pos[0]-imagePoint_neg[0])/(2*delta);
    numerical_pointJacobian(1,i)=(imagePoint_pos[1]-imagePoint_neg[1])/(2*delta);
  }
   
  //std::cout<<"Jacobian:\n"<<pointJacobian<<"\n";
  //std::cout<<"Numerical Jacobian:\n"<<numerical_pointJacobian<<"\n";
  // check that jacobians are within small threshold
  EXPECT_TRUE((pointJacobian-numerical_pointJacobian).cwiseAbs().sum()<1.0e-3);
}
TEST(PinholeCamera, Jacobian_w_Dist)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // create image point and jacobian
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;

  // project
  pinholeCamera.project(point_C,&imagePoint, &pointJacobian);

  // calculate jacobian via numeric differences
  Eigen::Matrix<double, 2, 3> numerical_pointJacobian;
  double delta=1e-4;
  for(int i=0; i<3;i++)
  {
    auto point_C_pos=point_C;
    auto point_C_neg=point_C;
    point_C_pos[i]+=delta;
    point_C_neg[i]-=delta;
    Eigen::Vector2d imagePoint_pos;
    Eigen::Vector2d imagePoint_neg;
    pinholeCamera.project(point_C_pos,&imagePoint_pos);
    pinholeCamera.project(point_C_neg,&imagePoint_neg);
    numerical_pointJacobian(0,i)=(imagePoint_pos[0]-imagePoint_neg[0])/(2*delta);
    numerical_pointJacobian(1,i)=(imagePoint_pos[1]-imagePoint_neg[1])/(2*delta);

  }
   
  //std::cout<<"Jacobian:\n"<<pointJacobian<<"\n";
  //std::cout<<"Numerical Jacobian:\n"<<numerical_pointJacobian<<"\n";
  // check that jacobians are within small threshold
  EXPECT_TRUE((pointJacobian-numerical_pointJacobian).cwiseAbs().sum()<1.0e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

