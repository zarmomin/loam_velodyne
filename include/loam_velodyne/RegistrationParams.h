//
// Created by nico on 26.11.18.
//

#ifndef LOAM_VELODYNE_REGISTRATIONPARAMS_H
#define LOAM_VELODYNE_REGISTRATIONPARAMS_H

#include "Eigen/Eigen"
#include "Vector3.h"
#include "yaml-cpp/yaml.h"

namespace loam {

/** Scan Registration configuration parameters. */
class RegistrationParams {
 public:

  RegistrationParams();

  RegistrationParams(const std::string& filename);

  void load(const std::string& filename);

  void loadSurfaceParameters(const YAML::Node &config);

  void loadTransformations(const YAML::Node &config);

  /** The time per scan. */
  float scanPeriod;

  /** The size of the IMU history state buffer. */
  int imuHistorySize;

  /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
  int nFeatureRegions;

  /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
  int curvatureRegion;

  /** The maximum number of sharp corner points per feature region. */
  int maxCornerSharp;

  /** The maximum number of less sharp corner points per feature region. */
  int maxCornerLessSharp;

  /** The maximum number of flat surface points per feature region. */
  int maxSurfaceFlat;

  /** The voxel size used for down sizing the remaining less flat surface points. */
  float lessFlatFilterSize;

  std::string outputFrame;

  /** The curvature threshold below / above a point is considered a flat / corner point. */
  float surfaceCurvatureThreshold;

  /** The homogeneous transform between the lidar and the camera frame */
  Eigen::Matrix4f T_camera_lidar;

  /** The homogeneous transform between the lidar and the camera frame */
  Eigen::Matrix4f T_camera_imu;
 private:
  /** Sets the default values per the original loam implementation. All frames originate at the same point and only vary by rotation. **/
  void SetDefaultValues();
};

}
#endif //LOAM_VELODYNE_REGISTRATIONPARAMS_H
