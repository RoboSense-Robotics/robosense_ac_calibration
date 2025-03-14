#include "lidar_calib.h"
#include "pca.h"

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include "common_function.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace lidar_calib {
LidarCalib::LidarCalib() {
}

LidarCalib::~LidarCalib() {
}

pcl::PointXYZI LidarCalib::calCloudCenterPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _points) {
  pcl::PointXYZI center_p;
  double center_p_x = 0;
  double center_p_y = 0;
  double center_p_z = 0;

  int point_num = 0;

  for (size_t i = 0; i < _points->size(); i++) {
    pcl::PointXYZI cur_p = _points->points.at(i);

    if (pcl::isFinite(cur_p)) {
      point_num++;
      center_p_x += cur_p.x;
      center_p_y += cur_p.y;
      center_p_z += cur_p.z;
    }
  }

  if (point_num > 0) {
    center_p.x = center_p_x / point_num;
    center_p.y = center_p_y / point_num;
    center_p.z = center_p_z / point_num;
  }

  return center_p;
}

Eigen::Vector3d LidarCalib::calPCANormalVector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points) {
  int point_num = _plane_points->size();
  Eigen::Vector3d normal_vec;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pca_data_matrix(point_num, 3);

  for (size_t i = 0; i < point_num; i++) {
    pcl::PointXYZI cur_p  = _plane_points->points.at(i);
    pca_data_matrix(i, 0) = cur_p.x;
    pca_data_matrix(i, 1) = cur_p.y;
    pca_data_matrix(i, 2) = cur_p.z;
  }

  PcaT<double> pca;
  pca.setInput(pca_data_matrix);
  pca.compute();

  return pca.getEigenVectors().col(2);
}

Eigen::Matrix4f LidarCalib::poseToMatrix(const std::vector<double>& _pose) {
  /// generate transform matrix according to current pose

  double roll  = _pose[3];
  double pitch = _pose[4];
  double yaw   = _pose[5];

  Eigen::AngleAxisf current_rotation_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf current_rotation_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf current_rotation_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f current_translation(_pose[0], _pose[1], _pose[2]);
  Eigen::Matrix4f trans = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

  return trans;
}

Eigen::Matrix4f LidarCalib::poseToMatrix(const double& _pose_x,
                                         const double& _pose_y,
                                         const double& _pose_z,
                                         const double& _pose_roll,
                                         const double& _pose_pitch,
                                         const double& _pose_yaw) {
  return poseToMatrix(std::vector<double> { _pose_x, _pose_y, _pose_z, _pose_roll, _pose_pitch, _pose_yaw });
}

std::vector<double> LidarCalib::calPlaneParams(const Eigen::Vector3d& _normal_vec, const pcl::PointXYZI& _plane_p) {

  return { _normal_vec[0], _normal_vec[1], _normal_vec[2],
           -(_normal_vec[0] * _plane_p.x + _normal_vec[1] * _plane_p.y + _normal_vec[2] * _plane_p.z) };
}

void LidarCalib::centralProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _raw_points_ptr,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points_ptr,
                                   std::vector<double>& _plane,
                                   const double& _fit_error = 0.1) {
  // _plane = [A, B, C, D]，表示空间平面的 Ax + By + Cz + D = 0
  // 理论上t的绝对值应该小于等于1，考虑到测距存在误差，暂时给0.1的余量
  double t_th = 1 + _fit_error;

  _plane_points_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  for (pcl::PointXYZI cur_p : *_raw_points_ptr) {
    pcl::PointXYZI plane_p;
    // 参数方程 Aat + Bbt + Cct + D = 0
    double t = -_plane[3] / (_plane[0] * cur_p.x + _plane[1] * cur_p.y + _plane[2] * cur_p.z);
    if (std::fabs(t) >= t_th) {
      continue;
    }
    plane_p.x         = cur_p.x * t;
    plane_p.y         = cur_p.y * t;
    plane_p.z         = cur_p.z * t;
    plane_p.intensity = cur_p.intensity;
    _plane_points_ptr->push_back(plane_p);
  }
}

void LidarCalib::cal3DPCANormalizedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _plane_points,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr& _normalized_cloud,
                                         std::vector<double>& _plane,
                                         const std::vector<double> _boundary_limit) {
  auto center_p = calCloudCenterPoint(_plane_points);  // 原始点云中心

  // 提取中间部分平整点云，计算法向量
  pcl::PointCloud<pcl::PointXYZI>::Ptr smooth_plane_points(new pcl::PointCloud<pcl::PointXYZI>);
  for (pcl::PointXYZI cur_p : *_plane_points) {
    if ((std::abs(cur_p.x - center_p.x) <= _boundary_limit[0] / 2) &&
        (std::abs(cur_p.y - center_p.y) <= _boundary_limit[1] / 2) &&
        (std::abs(cur_p.z - center_p.z) <= _boundary_limit[2] / 2)) {
      smooth_plane_points->push_back(cur_p);
    }
  }

  center_p = calCloudCenterPoint(smooth_plane_points);

  auto center_tf     = poseToMatrix(-center_p.x, -center_p.y, -center_p.z, 0, 0, 0);
  auto inv_center_tf = poseToMatrix(center_p.x, center_p.y, center_p.z, 0, 0, 0);

  // 中心位置归一化
  pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*_plane_points, *center_cloud, center_tf);

  pcl::PointCloud<pcl::PointXYZI>::Ptr center_plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*smooth_plane_points, *center_plane_cloud, center_tf);

  // 计算面的法向量
  Eigen::Vector3d normal_vec = calPCANormalVector(center_plane_cloud);
  // 根据法向量计算欧拉角
  double pitch              = std::atan(normal_vec(2) / normal_vec(0));
  double yaw                = std::atan(normal_vec(1) / normal_vec(0));
  Eigen::Matrix4f rotate_tf = poseToMatrix(0, 0, 0, 0, pitch, -yaw);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*center_cloud, *tf_cloud, rotate_tf);

  pcl::PointCloud<pcl::PointXYZI>::Ptr check_tf_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*tf_cloud, *check_tf_cloud, inv_center_tf);

  // 对完成旋转的点云进行中心投影变换
  _plane = calPlaneParams(normal_vec, center_p);
  centralProjection(_plane_points, _normalized_cloud, _plane);
}

void LidarCalib::segmentBoardPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_points_ptr) {
  if (segmented_points_ptr == nullptr) {
    segmented_points_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }

  // ROI过滤
  pcl::CropBox<pcl::PointXYZI> crop_box;
  crop_box.setInputCloud(input_points_ptr_);

  // 设置 ROI 区域的边界
  crop_box.setMin(min_pt_);  // 设置最小边界
  crop_box.setMax(max_pt_);  // 设置最大边界

  // 执行过滤，保存过滤后的点云
  crop_box.filter(*input_points_ptr_);
  pcl::io::savePCDFileBinary("/home/sti/0_super_sensor_sdks/open_source_sdks/src/calibration/debug/ROI_cloud.pcd",
                             *input_points_ptr_);

  // 点云分割
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型为平面
  seg.setMethodType(pcl::SAC_RANSAC);     // 使用RANSAC算法
  seg.setDistanceThreshold(0.02);         // 设置平面点与模型的最大距离

  // 创建一个指向分割结果的索引对象
  pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);

  // 创建一个存储分割平面的指针
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // 执行分割操作，找到平面模型
  seg.setInputCloud(input_points_ptr_);
  seg.segment(*inlierIndices, *coefficients);

  if (inlierIndices->indices.empty()) {
    std::cout << "Could not estimate a planar model for the given dataset.";
    return;
  }


  // 提取平面点和背景点
  pcl::ExtractIndices<pcl::PointXYZI> extract;

  // 提取平面点
  extract.setInputCloud(input_points_ptr_);
  extract.setIndices(inlierIndices);
  extract.setNegative(false);  // 提取平面点
  extract.filter(*segmented_points_ptr);

  segmented_points_ptr_ = segmented_points_ptr;
}

void LidarCalib::fittingPlaneBoardPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& fitted_points_ptr) {
  plane_.clear();
  std::vector<double> boundary_limit = { 0.5, 1, 1 };
  cal3DPCANormalizedCloud(segmented_points_ptr_, fitted_points_ptr, plane_, boundary_limit);

  fitted_points_ptr_ = fitted_points_ptr;
}

void LidarCalib::solveBoardPose(Eigen::Matrix4d& transform_mat,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& fitted_points_ptr) {

  auto center_p = calCloudCenterPoint(fitted_points_ptr_);

  normalizedPointsToVehicleCS(fitted_points_ptr_, transform_mat);
}

void LidarCalib::getInputPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_ptr) {
  input_points_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  *input_points_ptr_ = *_input_ptr;
}

void LidarCalib::normalizedPointsToVehicleCS(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_points_ptr,
                                             Eigen::Matrix4d& transform_mat) {

  Eigen::Matrix3d rot_matrix = transform_mat.block<3, 3>(0, 0);

  // 将旋转矩阵转换为轴角表示
  Eigen::AngleAxisd angle_axis(rot_matrix);

  // 计算旋转向量（轴乘以角度）
  Eigen::Vector3d rot_axis_vec = angle_axis.axis() * angle_axis.angle();

  auto center_p = calCloudCenterPoint(input_points_ptr);
  ceres::Problem problem;
  ceres::Solver::Options option;
  option.update_state_every_iteration = true;
  option.linear_solver_type           = ceres::DENSE_SCHUR;
  option.minimizer_progress_to_stdout = false;
  option.num_threads                  = 16;

  double ceres_r[3] { rot_axis_vec[0], rot_axis_vec[1], rot_axis_vec[2] };
  double ceres_t[3] { center_p.x, center_p.y, center_p.z };
  std::vector<double> upp_bound(3, M_PI / 1);
  std::vector<double> low_bound(3, -M_PI / 1);

  for (auto cur_p : *input_points_ptr) {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 3, 3>(new CostFunctor(cur_p));
    problem.AddResidualBlock(cost_function, nullptr, ceres_r, ceres_t);
  }

  for (int i = 0; i < 3; ++i) {
    problem.SetParameterLowerBound(ceres_r, i, low_bound[i]);
    problem.SetParameterUpperBound(ceres_r, i, upp_bound[i]);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);

  double rotation[9];
  ceres::AngleAxisToRotationMatrix<double>(ceres_r, rotation);
  transform_mat.setIdentity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transform_mat(j, i) = rotation[i * 3 + j];
    }
  }
  transform_mat(0, 3) = ceres_t[0];
  transform_mat(1, 3) = ceres_t[1];
  transform_mat(2, 3) = ceres_t[2];
  pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_points_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*input_points_ptr, *input_points_ptr, transform_mat);

  return;
}

void LidarCalib::setROIArea(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt) {
  min_pt_ = min_pt;
  max_pt_ = max_pt;
}

void LidarCalib::calibLidarExtrinsics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_ptr,
                                      Eigen::Matrix4d& lidar_board_transform) {
  Eigen::Matrix4d board_lidar_transform = lidar_board_transform.inverse();
  double chess_len                      = 0.108;

  Eigen::Matrix4d center_board_transform   = Eigen::Matrix4d::Identity();
  center_board_transform.block<3, 1>(0, 3) = Eigen::Vector3d(chess_len * 1.5, chess_len * 1.5, 0.0);

  // std::cout << "center_board_transform offset:\n" << center_board_transform << std::endl;

  Eigen::Matrix4d center_lidar_transform = board_lidar_transform * center_board_transform;

  Eigen::Vector3d t_vec      = center_lidar_transform.block<3, 1>(0, 3);
  Eigen::Matrix3d rot_matrix = center_lidar_transform.block<3, 3>(0, 0);

  auto estimate_center_lidar_transform = center_lidar_transform;

  double d_x = 0.25;
  double d_y = 0.5;
  double d_z = 0.5;

  min_pt_ = Eigen::Vector4f(t_vec[0] - d_x, t_vec[1] - d_y, t_vec[2] - d_z, 1.0);
  max_pt_ = Eigen::Vector4f(t_vec[0] + d_x, t_vec[1] + d_y, t_vec[2] + d_z, 1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_board_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr fitted_board_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  Eigen::Matrix4d lidar_center_transform = center_lidar_transform.inverse();
  getInputPointcloud(points_ptr);
  segmentBoardPoints(raw_board_pointcloud_ptr);
  fittingPlaneBoardPoints(fitted_board_pointcloud_ptr);
  solveBoardPose(lidar_center_transform, fitted_board_pointcloud_ptr);

  center_lidar_transform = lidar_center_transform.inverse();

  std::vector<double> offset_rtvec = { 0.0, -M_PI / 2, 0.0, chess_len * 1.5, chess_len * 1.5, 0.0 };
  Eigen::Matrix4d offset_transform = robosense::calib::factory_calibration::getMatrixFromEulerPose(offset_rtvec);
  lidar_board_transform  = offset_transform * lidar_center_transform;
}

}  // namespace lidar_calib