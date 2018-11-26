//
// Created by nico on 26.11.18.
//

#include <loam_velodyne/RegistrationParams.h>
#include "loam_velodyne/common.h"

namespace loam {

RegistrationParams::RegistrationParams() {
  SetDefaultValues();
}

RegistrationParams::RegistrationParams(const std::string &filename) {
  load(filename);
}

void RegistrationParams::SetDefaultValues() {
  scanPeriod = 0.1;
  imuHistorySize = 200;
  nFeatureRegions = 6;
  curvatureRegion = 5;
  maxCornerSharp = 2;
  maxSurfaceFlat = 4;
  outputFrame = "/camera";
  surfaceCurvatureThreshold = 0.1;
  T_camera_lidar << 0, 1, 0, 0,
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 0, 0, 1;
  T_camera_imu << 0, 1, 0, 0,
                  0, 0, 1, 0,
                  1, 0, 0, 0,
                  0, 0, 0, 1;
}

void RegistrationParams::load(const std::string &filename) {
  if (exists(filename)) {
    YAML::Node config = YAML::LoadFile(filename);
    loadSurfaceParameters(config);
    loadTransformations(config);
  }
  else
  {
    ROS_INFO("No configuration file found at %s\n Using default values instead.", filename.c_str());
    SetDefaultValues();
  }
}

void RegistrationParams::loadSurfaceParameters(const YAML::Node &config) {
  scanPeriod = config["scan_period"].as<float>();
  imuHistorySize = config["imu_history_size"].as<int>();
  nFeatureRegions = config["n_feature_regions"].as<int>();
  curvatureRegion = config["curvature_region"].as<int>();
  maxCornerSharp = config["max_corner_sharp"].as<int>();
  maxCornerLessSharp = 10 * maxCornerSharp;
  maxSurfaceFlat = config["max_surface_flat"].as<int>();
  outputFrame = config["output_frame"].as<std::string>();
  surfaceCurvatureThreshold = config["surface_curvature_threshold"].as<float>();
}

void RegistrationParams::loadTransformations(const YAML::Node &config) {
  T_camera_lidar(0,0) = config["T_camera_lidar"]["data"][0].as<float>();
  T_camera_lidar(0,1) = config["T_camera_lidar"]["data"][1].as<float>();
  T_camera_lidar(0,2) = config["T_camera_lidar"]["data"][2].as<float>();
  T_camera_lidar(0,3) = config["T_camera_lidar"]["data"][3].as<float>();

  T_camera_lidar(1,0) = config["T_camera_lidar"]["data"][4].as<float>();
  T_camera_lidar(1,1) = config["T_camera_lidar"]["data"][5].as<float>();
  T_camera_lidar(1,2) = config["T_camera_lidar"]["data"][6].as<float>();
  T_camera_lidar(1,3) = config["T_camera_lidar"]["data"][7].as<float>();

  T_camera_lidar(2,0) = config["T_camera_lidar"]["data"][8].as<float>();
  T_camera_lidar(2,1) = config["T_camera_lidar"]["data"][9].as<float>();
  T_camera_lidar(2,2) = config["T_camera_lidar"]["data"][10].as<float>();
  T_camera_lidar(2,3) = config["T_camera_lidar"]["data"][11].as<float>();

  T_camera_lidar(3,0) = config["T_camera_lidar"]["data"][12].as<float>();
  T_camera_lidar(3,1) = config["T_camera_lidar"]["data"][13].as<float>();
  T_camera_lidar(3,2) = config["T_camera_lidar"]["data"][14].as<float>();
  T_camera_lidar(3,3) = config["T_camera_lidar"]["data"][15].as<float>();

  T_camera_imu(0,0) = config["T_camera_imu"]["data"][0].as<float>();
  T_camera_imu(0,1) = config["T_camera_imu"]["data"][1].as<float>();
  T_camera_imu(0,2) = config["T_camera_imu"]["data"][2].as<float>();
  T_camera_imu(0,3) = config["T_camera_imu"]["data"][3].as<float>();

  T_camera_imu(1,0) = config["T_camera_imu"]["data"][4].as<float>();
  T_camera_imu(1,1) = config["T_camera_imu"]["data"][5].as<float>();
  T_camera_imu(1,2) = config["T_camera_imu"]["data"][6].as<float>();
  T_camera_imu(1,3) = config["T_camera_imu"]["data"][7].as<float>();

  T_camera_imu(2,0) = config["T_camera_imu"]["data"][8].as<float>();
  T_camera_imu(2,1) = config["T_camera_imu"]["data"][9].as<float>();
  T_camera_imu(2,2) = config["T_camera_imu"]["data"][10].as<float>();
  T_camera_imu(2,3) = config["T_camera_imu"]["data"][11].as<float>();

  T_camera_imu(3,0) = config["T_camera_imu"]["data"][12].as<float>();
  T_camera_imu(3,1) = config["T_camera_imu"]["data"][13].as<float>();
  T_camera_imu(3,2) = config["T_camera_imu"]["data"][14].as<float>();
  T_camera_imu(3,3) = config["T_camera_imu"]["data"][15].as<float>();
}


}