/******************************************************************************
   Copyright 2025 RoboSense Technology Co., Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 *****************************************************************************/

#ifndef lidar_calib_H
#define lidar_calib_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace lidar_calib {

struct CostFunctor {
  CostFunctor(const pcl::PointXYZI& _point) : point_(_point) {
  }

  template <typename T>
  bool operator()(const T* const ceres_r, const T* const ceres_t, T* residual) const {
    T p_real[3] = { T(point_.x), T(point_.y), T(point_.z) };
    T p_perfect[3];
    ceres::AngleAxisRotatePoint(ceres_r, p_real, p_perfect);
    p_perfect[0] = p_perfect[0] + ceres_t[0];
    p_perfect[1] = p_perfect[1] + ceres_t[1];
    p_perfect[2] = p_perfect[2] + ceres_t[2];

    T board_half_width  = T(0.6 / 2.0);
    T board_half_height = T(0.6 / 2.0);
    T thickness_epsilon = T(0.05);

    T c_x = ceres::abs(p_perfect[0]);
    // T c_x =
    //   ceres::abs(p_perfect[0]) > thickness_epsilon ?
    //   ceres::abs(ceres::abs(p_perfect[0]) - thickness_epsilon) : T(0.0);
    T c_y =
      ceres::abs(p_perfect[1]) > board_half_width ? ceres::abs(ceres::abs(p_perfect[1]) - board_half_width) : T(0.0);
    T c_z =
      ceres::abs(p_perfect[2]) > board_half_height ? ceres::abs(ceres::abs(p_perfect[2]) - board_half_height) : T(0.0);

    residual[0] = c_x;
    residual[1] = c_y;
    residual[2] = c_z;
    return true;
  }

private:
  const pcl::PointXYZI point_;
};

class LidarCalib {
private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_points_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr segmented_points_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fitted_points_ptr_;
  std::vector<double> plane_;

  Eigen::Vector4f min_pt_;
  Eigen::Vector4f max_pt_;

  pcl::PointXYZI calCloudCenterPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _points);

  Eigen::Vector3d calPCANormalVector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points);

  Eigen::Matrix4f poseToMatrix(const std::vector<double>& _pose);

  Eigen::Matrix4f poseToMatrix(const double& _pose_x,
                               const double& _pose_y,
                               const double& _pose_z,
                               const double& _pose_roll,
                               const double& _pose_pitch,
                               const double& _pose_yaw);

  std::vector<double> calPlaneParams(const Eigen::Vector3d& _normal_vec, const pcl::PointXYZI& _plane_p);

  void centralProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _raw_points_ptr,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points_ptr,
                         std::vector<double>& _plane,
                         const double& _fit_error);

  void normalizedPointsToVehicleCS(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_points_ptr,
                                   Eigen::Matrix4d& transform_mat);

  void cal3DPCANormalizedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr& _normalized_cloud,
                               std::vector<double>& _plane,
                               const std::vector<double> _boundary_limit);

public:
  LidarCalib();
  ~LidarCalib();
  void getInputPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_ptr);
  void setROIArea(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt);

  void segmentBoardPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_points_ptr);
  void fittingPlaneBoardPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& fitted_points_ptr);
  void solveBoardPose(Eigen::Matrix4d& transform_mat, pcl::PointCloud<pcl::PointXYZI>::Ptr& fitted_points_ptr);

  void calibLidarExtrinsics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_ptr, Eigen::Matrix4d& lidar_pose);
};
}  // namespace lidar_calib
#endif
