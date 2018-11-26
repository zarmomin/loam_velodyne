// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/ScanRegistration.h"
#include <tf/transform_datatypes.h>
#include "eigen_conversions/eigen_msg.h"
#include "loam_velodyne/RegistrationParams.h"
#include "loam_velodyne/math_utils.h"
#include "ros/ros.h"
#include <exception>

namespace loam {

bool ScanRegistration::parseParams(const ros::NodeHandle& nh,
                                   RegistrationParams& config_out) {
  bool success = true;
  try{
    std::string configFile;
    if (nh.getParam("config_file", configFile)) {
      config_out = RegistrationParams(configFile);
      ROS_INFO("Used config from file: %s", configFile.c_str());
    }
    else{
      config_out = RegistrationParams();
      ROS_INFO("Parameter 'config_file' was not set. Will use default configuration.");
    }
  } catch (std::exception ex) {
    ROS_ERROR("Error when trying to read config file: %s", ex.what());
    success = false;
  }
  return success;
}

bool ScanRegistration::setupROS(ros::NodeHandle& node,
                                ros::NodeHandle& privateNode,
                                RegistrationParams& config_out) {
  if (!parseParams(privateNode, config_out)) return false;

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>(
      "/imu/data", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  _pubLaserCloud =
      node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}

void ScanRegistration::handleIMUMessage(
    const sensor_msgs::Imu::ConstPtr& imuIn) {
  // use tf to get RPY angles, since the results are nicer (i.e. closer to 0)
  // than the Eigen version
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // transform & remove gravity acceleration with the following frames
  // I:imu frame, C:camera frame, W:world frame
  // C_acc = r_CI * I_a - r_CI * RPY' * W_gravity
  // the fourth component of the gravity vector and acceleration are set to 0 to
  // ensure that the translational part
  // of the homogeneous transform T_camera_imu does not get taken into account

  Eigen::Quaterniond orientationIMU;
  tf::quaternionMsgToEigen(imuIn->orientation, orientationIMU);
  Eigen::Matrix4f rpy = Eigen::Affine3f(orientationIMU.cast<float>()).matrix();
  Eigen::Vector4f acceleration(imuIn->linear_acceleration.x,
                               imuIn->linear_acceleration.y,
                               imuIn->linear_acceleration.z, 0);
  Vector3 acceleration_camera =
      config().T_camera_imu * acceleration -
      config().T_camera_imu * rpy.transpose() * Eigen::Vector4f(0, 0, 9.81f, 0);

  IMUState newState;
  newState.stamp = fromROSTime(imuIn->header.stamp);
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acceleration_camera;

  updateIMUData(newState);
}

void ScanRegistration::publishResult() {
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime,
                  config().outputFrame);
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime,
                  config().outputFrame);
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(),
                  sweepStartTime, config().outputFrame);
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime,
                  config().outputFrame);
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(),
                  sweepStartTime, config().outputFrame);

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime,
                  config().outputFrame);
}

}  // end namespace loam
