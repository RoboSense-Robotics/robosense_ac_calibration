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

#ifndef CALIB_CORE_H
#define CALIB_CORE_H

#include "camera_calib.h"
#include "lidar_calib.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stack>
#include <yaml-cpp/yaml.h>

enum CalibStatus {
  None      = 0,
  CameraInt = 1,
  Ext       = 2,
};

struct extrinsic {
  std::vector<double> translation;
  std::vector<double> quaternion;  // w, x, y, z
  Eigen::Matrix4d transform_matrix;
};

struct ExtrinsicCalibrationResidual {
  ExtrinsicCalibrationResidual(const Eigen::Matrix4d& camera_pose, const Eigen::Matrix4d& lidar_pose) :
    C_(camera_pose), L_(lidar_pose) {
  }

  template <typename T>
  bool operator()(const T* const extrinsic_params, T* residual) const {
    // Extract rotation and translation from camera and lidar poses
    Eigen::Matrix<T, 3, 3> R_C = C_.block<3, 3>(0, 0).cast<T>();
    Eigen::Matrix<T, 3, 1> t_C = C_.block<3, 1>(0, 3).cast<T>();
    Eigen::Matrix<T, 3, 3> R_L = L_.block<3, 3>(0, 0).cast<T>();
    Eigen::Matrix<T, 3, 1> t_L = L_.block<3, 1>(0, 3).cast<T>();

    // Parse parameters: rotation vector (axis-angle) and translation
    Eigen::Matrix<T, 3, 1> axis_angle;
    axis_angle << extrinsic_params[0], extrinsic_params[1], extrinsic_params[2];
    T angle = axis_angle.norm();

    // Compute rotation matrix from axis-angle
    Eigen::Matrix<T, 3, 3> R_T_cl;
    if (angle < T(1e-8)) {
      R_T_cl = Eigen::Matrix<T, 3, 3>::Identity();
    } else {
      Eigen::Matrix<T, 3, 1> axis = axis_angle / angle;
      R_T_cl                      = Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
    }

    // Predicted camera rotation: R_T_cl * R_L
    Eigen::Matrix<T, 3, 3> R_pred = R_T_cl * R_L;

    // Rotation residual: log(R_C^T * R_pred)
    Eigen::Matrix<T, 3, 3> R_diff = R_C.transpose() * R_pred;
    Eigen::AngleAxis<T> aa(R_diff);
    Eigen::Matrix<T, 3, 1> rot_residual = aa.angle() * aa.axis();

    // Translation residual: t_C - (R_T_cl * t_L + t_T_cl)
    Eigen::Matrix<T, 3, 1> t_T_cl(extrinsic_params[3], extrinsic_params[4], extrinsic_params[5]);
    Eigen::Matrix<T, 3, 1> t_pred         = R_T_cl * t_L + t_T_cl;
    Eigen::Matrix<T, 3, 1> trans_residual = t_C - t_pred;

    // Assign residuals
    for (int i = 0; i < 3; ++i) {
      residual[i]     = rot_residual[i];
      residual[i + 3] = trans_residual[i];
    }
    return true;
  }

private:
  const Eigen::Matrix4d C_;
  const Eigen::Matrix4d L_;
};

class CalibCore {
private:
  void calibrateCameraIntrinsics();
  void calibrateExtrinsics();

  void loadCalibrationParameters(const std::string& _config_file);
  void loadExtrinsic(const YAML::Node& _extrinsic_node, extrinsic& _extrinsic);

  void calibrateSingleCameraLidarExtrinsics(const cv::Mat& img,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                                            Eigen::Matrix4d& camera_pose,
                                            Eigen::Matrix4d& lidar_pose);
  void calibrateSingleCameraExtrinsics(const cv::Mat& img, Eigen::Matrix4d& camera_pose);
  void calibrateSingleLidarExtrinsics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                                      Eigen::Matrix4d& lidar_pose);
  Eigen::Matrix4d calibrateCameraLidarExtrinsics(const std::vector<Eigen::Matrix4d>& camera_pose_vec,
                                                 const std::vector<Eigen::Matrix4d>& lidar_pose_vec);
  Eigen::Matrix4d calibrateCameraImuExtrinsics(const std::vector<Eigen::Matrix4d>& camera_delta_pose_vec,
                                               const std::vector<Eigen::Matrix4d>& imu_delta_pose_vec);

  std::vector<Eigen::Matrix4d> integrateIMU(const std::queue<sensor_msgs::msg::Imu::SharedPtr>& imu_queue,
                                            const std::vector<double>& static_pose_timestamp_vec);

  bool checkMotion(const sensor_msgs::msg::Imu::SharedPtr msg, double angular_thresh);

  void solveHandEyeCalibration(const std::vector<Eigen::Matrix4d>& matrix_A,
                               const std::vector<Eigen::Matrix4d>& matrix_B,
                               Eigen::Matrix4d& _transform_X);
  std::string getCameraLidarCalibResult();

  size_t current_pose_;
  size_t current_index_;

  std::stack<cv::Mat> img_stack_;

  std::shared_ptr<lidar_calib::LidarCalib> lidar_calib_ptr_;
  std::shared_ptr<camera_calib::CameraCalib> camera_calib_ptr_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_pointcloud_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_board_pointcloud_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fitted_board_pointcloud_ptr_;

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_msg_queue_;
  std::stack<sensor_msgs::msg::Imu::SharedPtr> imu_msg_stack_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_vec_;
  std::vector<sensor_msgs::msg::Image::SharedPtr> image_msg_vec_;
  std::vector<double> static_pose_timestamp_vec_;
  double last_motion_time_ = 0.0;
  bool silence_detected_   = false;

  size_t pointcloud_count_ = 0;

  Eigen::Vector4f min_pt_;
  Eigen::Vector4f max_pt_;
  Eigen::Matrix4d lidar_board_pose_;

  std::string config_file_;

  std::atomic<bool> is_ext_calib_ready_;
  size_t ext_calib_pose_count_ = 0;

  extrinsic camera_extrinsic_;
  extrinsic lidar_extrinsic_;
  extrinsic imu_extrinsic_;

  std::string camera_model_;
  cv::Mat camera_int_matrix_;
  cv::Mat camera_dist_coeff_;
  cv::Size camera_image_size_;

  CalibStatus calib_status_ = CalibStatus::None;

public:
  CalibCore();
  ~CalibCore();
  void setConfigFile(const std::string& _config_file);

  void initCameraConfig();

  void startCalib(const CalibStatus calib_status);
  bool autoPushImage(cv::Mat& img);
  void addBoardIndicator(cv::Mat& img);

  void setROIArea(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt);
  void lidarCalibrate();

  pcl::PointCloud<pcl::PointXYZI>::Ptr getRawBoardPoints();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getFittedBoardPoints();
  size_t getImageCount();
  std::string getCalibResult();

  bool isExtCalibReady();
  void setExtCalibReady(const bool status);
  void addExtCalibPose();
  size_t getExtCalibPose();
  void clearExtCalibPose();

  void setImageMsgInput(const sensor_msgs::msg::Image::SharedPtr& msg);
  void setImuMsgInput(const sensor_msgs::msg::Imu::SharedPtr& msg);
  void setPointcloudInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr);
};

#endif