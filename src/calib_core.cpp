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

#include "calib_core.h"
#include "common_function.h"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>

using namespace robosense::calib;

CalibCore::CalibCore() {
  camera_calib_ptr_.reset(new camera_calib::CameraCalib);
  is_ext_calib_ready_ = true;
}

CalibCore::~CalibCore() {
}

void CalibCore::setConfigFile(const std::string& _config_file) {
  config_file_ = _config_file;
  loadCalibrationParameters(config_file_);
  return;
}

void CalibCore::loadCalibrationParameters(const std::string& _config_file) {
  if (access(_config_file.c_str(), F_OK) == -1) {
    std::cout << "Camera config file does not exist!!!      " << _config_file << std::endl;
    return;
  }
  YAML::Node doc              = YAML::LoadFile(_config_file);
  auto calibraion_result_file = doc["calibration_result"].as<std::string>();

  if (access(calibraion_result_file.c_str(), F_OK) == -1) {
    std::cout << "calibraion result file does not exist!!!      " << calibraion_result_file << std::endl;
    return;
  }
  YAML::Node calibraion_result_doc = YAML::LoadFile(calibraion_result_file)["Sensor"];

  loadExtrinsic(calibraion_result_doc["Camera"]["extrinsic"], camera_extrinsic_);
  loadExtrinsic(calibraion_result_doc["Lidar"]["extrinsic"], lidar_extrinsic_);
  loadExtrinsic(calibraion_result_doc["IMU"]["extrinsic"], imu_extrinsic_);

  auto intrinsic   = calibraion_result_doc["Camera"]["intrinsic"]["int_matrix"].as<std::vector<double>>();
  auto image_size  = calibraion_result_doc["Camera"]["intrinsic"]["image_size"].as<std::vector<int>>();
  auto distor_coff = calibraion_result_doc["Camera"]["intrinsic"]["dist_coeff"].as<std::vector<double>>();

  camera_model_      = calibraion_result_doc["Camera"]["intrinsic"]["model"].as<std::string>();
  camera_int_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_dist_coeff_ = cv::Mat::eye(1, 14, CV_64F);
  camera_image_size_ = cv::Size(image_size[0], image_size[1]);

  if (intrinsic.size() != 9) {
    std::cout << "intrinsic size error" << std::endl;
  } else {
    camera_int_matrix_.at<double>(0, 0) = intrinsic[0];
    camera_int_matrix_.at<double>(0, 2) = intrinsic[2];
    camera_int_matrix_.at<double>(1, 1) = intrinsic[4];
    camera_int_matrix_.at<double>(1, 2) = intrinsic[5];
  }

  for (int i = 0; i < 14; i++) {
    camera_dist_coeff_.at<double>(0, i) = i < distor_coff.size() ? distor_coff[i] : 0.0;
  }
  std::cout << "camera_calib_ptr_->setCameraModel" << std::endl;
  camera_calib_ptr_->setCameraModel(camera_model_, camera_int_matrix_, camera_dist_coeff_, camera_image_size_);

  initCameraConfig();
  return;
}

void CalibCore::loadExtrinsic(const YAML::Node& _extrinsic_node, extrinsic& _extrinsic) {
  std::cout << _extrinsic_node["translation"]["x"].as<double>() << std::endl;

  std::vector<double> translation = { _extrinsic_node["translation"]["x"].as<double>(),
                                      _extrinsic_node["translation"]["y"].as<double>(),
                                      _extrinsic_node["translation"]["z"].as<double>() };
  std::vector<double> quaternion  = { _extrinsic_node["quaternion"]["w"].as<double>(),
                                     _extrinsic_node["quaternion"]["x"].as<double>(),
                                     _extrinsic_node["quaternion"]["y"].as<double>(),
                                     _extrinsic_node["quaternion"]["z"].as<double>() };

  Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  transform_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);

  _extrinsic.translation      = translation;
  _extrinsic.quaternion       = quaternion;
  _extrinsic.transform_matrix = transform_matrix;
  return;
}

void CalibCore::initCameraConfig() {

  camera_calib_ptr_->loadConfig(config_file_);
  camera_calib_ptr_->initDetector();
  return;
}

void CalibCore::calibrateCameraIntrinsics() {
  auto start = std::chrono::high_resolution_clock::now();
  camera_calib_ptr_->startCalib();

  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "calib time cost:    " << std::chrono::duration<double>(end - start).count() << " s" << std::endl;
  std::cout << "===========================================" << std::endl;

  img_stack_     = std::stack<cv::Mat>();
  current_pose_  = img_stack_.size() / 6;
  current_index_ = img_stack_.size() % 6;
  return;
}

void CalibCore::calibrateExtrinsics() {

  std::cout << "static_pose_timestamp_vec: " << static_pose_timestamp_vec_.size() << std::endl;

  std::vector<Eigen::Matrix4d> camera_lidar_pose_vec;
  std::vector<Eigen::Matrix4d> camera_pose_vec, lidar_pose_vec;
  for (size_t i = 0; i < static_pose_timestamp_vec_.size(); ++i) {
    cv::Mat image = cv_bridge::toCvCopy(image_msg_vec_[i], image_msg_vec_[i]->encoding)->image;

    auto cloud_ptr = pointcloud_vec_[i];
    Eigen::Matrix4d camera_pose, lidar_pose;
    calibrateSingleCameraLidarExtrinsics(image, cloud_ptr, camera_pose, lidar_pose);
    camera_lidar_pose_vec.push_back(lidar_pose.inverse() * camera_pose);
    camera_pose_vec.push_back(camera_pose);
    lidar_pose_vec.push_back(lidar_pose);
  }

  Eigen::Matrix4d camera_imu_transform = Eigen::Matrix4d::Identity();
  if (lidar_pose_vec.size() < 3) {
    // 数据不足，无法进行手眼标定
    camera_imu_transform = camera_extrinsic_.transform_matrix;
  } else {
    std::vector<Eigen::Matrix4d> imu_delta_pose = integrateIMU(imu_msg_queue_, static_pose_timestamp_vec_);
    std::vector<Eigen::Matrix4d> camera_delta_pose;
    for (size_t i = 0; i < static_pose_timestamp_vec_.size() - 1; ++i) {
      camera_delta_pose.push_back(camera_pose_vec[i + 1] * camera_pose_vec[i].inverse());
      std::cout << "camera_delta_pose:\n" << camera_pose_vec[i + 1] * camera_pose_vec[i].inverse() << std::endl;
    }
    std::cout << "pose count: " << camera_delta_pose.size() << " -> " << imu_delta_pose.size() << std::endl;
    auto imu_camera_transform = calibrateCameraImuExtrinsics(imu_delta_pose, camera_delta_pose);
    camera_imu_transform      = imu_camera_transform.inverse();
  }

  auto camera_lidar_transform = calibrateCameraLidarExtrinsics(camera_pose_vec, lidar_pose_vec);
  // camera_lidar_transform      = camera_lidar_pose_vec[0];
  auto camera_lidar_pose   = robosense::calib::factory_calibration::getPoseFromMatrix(camera_imu_transform);
  auto lidar_imu_transform = camera_imu_transform * camera_lidar_transform.inverse();

  camera_extrinsic_.transform_matrix = camera_imu_transform;
  lidar_extrinsic_.transform_matrix  = lidar_imu_transform;
}

void CalibCore::calibrateSingleCameraExtrinsics(const cv::Mat& img, Eigen::Matrix4d& camera_pose) {
  camera_calib_ptr_->calibCameraExtrinsics(img, camera_pose);
}

void CalibCore::calibrateSingleLidarExtrinsics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                                               Eigen::Matrix4d& lidar_pose) {
  while (lidar_calib_ptr_ == nullptr) {
    std::cout << "LidarCalib pointer is null!" << std::endl;
    lidar_calib_ptr_.reset(new lidar_calib::LidarCalib);
  }
  lidar_calib_ptr_->calibLidarExtrinsics(cloud_ptr, lidar_pose);
}

void CalibCore::calibrateSingleCameraLidarExtrinsics(const cv::Mat& img,
                                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                                                     Eigen::Matrix4d& camera_pose,
                                                     Eigen::Matrix4d& lidar_pose) {
  calibrateSingleCameraExtrinsics(img, camera_pose);

  auto lidar_camera_transform = (camera_extrinsic_.transform_matrix).inverse() * lidar_extrinsic_.transform_matrix;
  lidar_pose                  = camera_pose * lidar_camera_transform;
  calibrateSingleLidarExtrinsics(cloud_ptr, lidar_pose);

  return;
}

Eigen::Matrix4d CalibCore::calibrateCameraLidarExtrinsics(const std::vector<Eigen::Matrix4d>& camera_pose_vec,
                                                          const std::vector<Eigen::Matrix4d>& lidar_pose_vec) {
  if (camera_pose_vec.empty()) {
    return Eigen::Matrix4d::Identity();
  }

  if (camera_pose_vec.size() < 2) {
    return lidar_pose_vec[0].inverse() * camera_pose_vec[0];
  }

  // Initialize parameters using first pair
  Eigen::Matrix4d T_cl_init = lidar_pose_vec[0].inverse() * camera_pose_vec[0];

  std::cout << "T_cl_init transform:\n" << T_cl_init << std::endl;

  Eigen::Matrix3d R_init = T_cl_init.block<3, 3>(0, 0);
  Eigen::Vector3d t_init = T_cl_init.block<3, 1>(0, 3);
  Eigen::AngleAxisd aa(R_init);
  Eigen::Vector3d axis_angle_init = aa.angle() * aa.axis();

  double params[6] = {
    axis_angle_init.x(), axis_angle_init.y(), axis_angle_init.z(), t_init.x(), t_init.y(), t_init.z()
  };

  // Build optimization problem
  ceres::Problem problem;
  for (size_t i = 0; i < camera_pose_vec.size(); ++i) {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ExtrinsicCalibrationResidual, 6, 6>(
      new ExtrinsicCalibrationResidual(camera_pose_vec[i], lidar_pose_vec[i]));
    problem.AddResidualBlock(cost_function, nullptr, params);
  }

  // Configure and solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type           = ceres::DENSE_SCHUR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Convert optimized parameters to matrix
  Eigen::Vector3d axis_angle(params[0], params[1], params[2]);
  double angle = axis_angle.norm();
  Eigen::Matrix3d R =
    angle < 1e-8 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(angle, axis_angle / angle).toRotationMatrix();
  Eigen::Vector3d t(params[3], params[4], params[5]);

  Eigen::Matrix4d T_cl   = Eigen::Matrix4d::Identity();
  T_cl.block<3, 3>(0, 0) = R;
  T_cl.block<3, 1>(0, 3) = t;

  return T_cl;
}

Eigen::Matrix4d CalibCore::calibrateCameraImuExtrinsics(const std::vector<Eigen::Matrix4d>& camera_delta_pose_vec,
                                                        const std::vector<Eigen::Matrix4d>& imu_delta_pose_vec) {
  Eigen::Matrix4d imu_camera_transform = Eigen::Matrix4d::Identity();
  solveHandEyeCalibration(camera_delta_pose_vec, imu_delta_pose_vec, imu_camera_transform);  // TODO: attention
  return imu_camera_transform;
}

std::vector<Eigen::Matrix4d> CalibCore::integrateIMU(const std::queue<sensor_msgs::msg::Imu::SharedPtr>& imu_queue,
                                                     const std::vector<double>& static_pose_timestamp_vec) {
  std::cout << "integrateIMU: " << imu_queue.size() << " -> " << static_pose_timestamp_vec.size() << std::endl;

  std::vector<Eigen::Matrix4d> result;

  // 将imu_queue拷贝到vector中以便处理
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_vec;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> temp_queue =
    imu_queue;  // 注意：这里需要原队列非const才能拷贝，实际可能需要其他处理方式
  while (!temp_queue.empty()) {
    imu_vec.push_back(temp_queue.front());
    temp_queue.pop();
  }

  // 按时间戳排序imu_vec
  std::sort(imu_vec.begin(), imu_vec.end(),
            [](const sensor_msgs::msg::Imu::SharedPtr& a, const sensor_msgs::msg::Imu::SharedPtr& b) {
              double t_a = a->header.stamp.sec + a->header.stamp.nanosec * 1e-9;
              double t_b = b->header.stamp.sec + b->header.stamp.nanosec * 1e-9;
              return t_a < t_b;
            });

  for (size_t i = 0; i < static_pose_timestamp_vec.size() - 1; ++i) {
    double start_time = static_pose_timestamp_vec[i];
    double end_time   = static_pose_timestamp_vec[i + 1];

    // 找到在时间段内的IMU数据
    auto start_it = std::lower_bound(imu_vec.begin(), imu_vec.end(), start_time,
                                     [](const sensor_msgs::msg::Imu::SharedPtr& imu, double t) {
                                       double imu_time = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
                                       return imu_time < t;
                                     });
    auto end_it   = std::upper_bound(imu_vec.begin(), imu_vec.end(), end_time,
                                     [](double t, const sensor_msgs::msg::Imu::SharedPtr& imu) {
                                     double imu_time = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
                                     return t < imu_time;
                                     });

    std::vector<sensor_msgs::msg::Imu::SharedPtr> segment(start_it, end_it);

    if (segment.empty()) {
      result.push_back(Eigen::Matrix4d::Identity());
      continue;
    }

    // 初始重力取自第一个IMU数据（假设设备在start_time静止）
    Eigen::Vector3d g_imu(segment.front()->linear_acceleration.x, segment.front()->linear_acceleration.y,
                          segment.front()->linear_acceleration.z);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double t_prev     = start_time;

    for (const auto& imu : segment) {
      double current_time = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
      double dt           = current_time - t_prev;
      if (dt <= 0.0) {
        continue;  // 忽略无效时间差
      }

      // 积分旋转
      Eigen::Vector3d ang_vel(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
      double theta = ang_vel.norm() * dt;
      Eigen::Quaterniond delta_q;
      if (theta < 1e-6) {
        delta_q = Eigen::Quaterniond::Identity();
      } else {
        delta_q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, ang_vel.normalized()));
      }
      Eigen::Quaterniond q(R);
      q = q * delta_q;
      q.normalize();
      R = q.toRotationMatrix();

      // 处理加速度
      Eigen::Vector3d accel(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
      Eigen::Vector3d accel_body  = accel - g_imu;
      Eigen::Vector3d accel_world = R * accel_body;

      // 积分速度
      v += accel_world * dt;
      // 积分位移
      p += v * dt + 0.5 * accel_world * dt * dt;

      t_prev = current_time;
    }

    // 构造变换矩阵
    Eigen::Matrix4d T   = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    result.push_back(T);
    std::cout << "imu_delta_pose:\n" << T << std::endl;
  }

  return result;
}

void CalibCore::startCalib(const CalibStatus calib_status) {
  calib_status_ = calib_status;
  switch (calib_status) {
  case CalibStatus::None: break;
  case CalibStatus::CameraInt: calibrateCameraIntrinsics(); break;
  case CalibStatus::Ext: calibrateExtrinsics(); break;
  default: break;
  }
}

bool CalibCore::autoPushImage(cv::Mat& img) {
  return camera_calib_ptr_->autoPushImage(img);
}

size_t CalibCore::getImageCount() {
  return camera_calib_ptr_->getImageCount();
}

void CalibCore::setPointcloudInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_ptr) {
  pointcloud_vec_.push_back(_input_ptr);
}

void CalibCore::setImuMsgInput(const sensor_msgs::msg::Imu::SharedPtr& msg) {
  imu_msg_queue_.push(msg);

  double angular_thresh = 0.05;  // unit: rad/s
  double current_stamp =
    static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  const bool is_moving = checkMotion(msg, angular_thresh);
  last_motion_time_    = last_motion_time_ < 1e-3 ? current_stamp : last_motion_time_;

  if (is_moving) {
    last_motion_time_ = current_stamp;
    silence_detected_ = false;
  } else {
    double silence_duration = current_stamp - last_motion_time_;

    if (silence_duration >= 1.0 && !silence_detected_) {
      silence_detected_ = true;
      // static_pose_timestamp_vec_.push_back(current_stamp);
      std::cout << "static_pose_timestamp: " << std::fixed << std::setprecision(3) << current_stamp << std::endl;
      is_ext_calib_ready_ = true;
    }
  }
  return;
}

bool CalibCore::checkMotion(const sensor_msgs::msg::Imu::SharedPtr msg, double angular_thresh) {
  // 检查角速度
  const double angular_velocity = std::sqrt(
    std::pow(msg->angular_velocity.x, 2) + std::pow(msg->angular_velocity.y, 2) + std::pow(msg->angular_velocity.z, 2));
  return angular_velocity > angular_thresh;
}

void CalibCore::setImageMsgInput(const sensor_msgs::msg::Image::SharedPtr& msg) {
  auto image_msg  = msg;
  image_msg->step = image_msg->width * 3;
  image_msg_vec_.push_back(image_msg);
  static_pose_timestamp_vec_.push_back(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  return;
}

void CalibCore::setROIArea(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt) {
  min_pt_ = min_pt;
  max_pt_ = max_pt;
}

void CalibCore::lidarCalibrate() {
  if (input_pointcloud_ptr_ == nullptr || input_pointcloud_ptr_->empty()) {
    std::cout << "pointcloud is empty..." << std::endl;
    return;
  }
  if (input_pointcloud_ptr_->size() < 60000) {
    return;
  }

  while (lidar_calib_ptr_ == nullptr) {
    std::cout << "LidarCalib pointer is null!" << std::endl;
    lidar_calib_ptr_.reset(new lidar_calib::LidarCalib);
  }
  raw_board_pointcloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  lidar_calib_ptr_->getInputPointcloud(input_pointcloud_ptr_);
  lidar_calib_ptr_->setROIArea(min_pt_, max_pt_);
  lidar_calib_ptr_->segmentBoardPoints(raw_board_pointcloud_ptr_);
  lidar_calib_ptr_->fittingPlaneBoardPoints(fitted_board_pointcloud_ptr_);
  lidar_calib_ptr_->solveBoardPose(lidar_board_pose_, fitted_board_pointcloud_ptr_);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CalibCore::getRawBoardPoints() {
  return raw_board_pointcloud_ptr_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CalibCore::getFittedBoardPoints() {
  return fitted_board_pointcloud_ptr_;
}

void CalibCore::addBoardIndicator(cv::Mat& img) {
  camera_calib_ptr_->addBoardIndicator(img);
}

std::string CalibCore::getCameraLidarCalibResult() {
  std::stringstream res_str;
  res_str << "calibration finished.\n";

  auto camera_imu_pose = robosense::calib::factory_calibration::getPoseFromMatrix(camera_extrinsic_.transform_matrix);

  res_str << "camera extrinsic:\n";
  res_str << std::fixed << std::setprecision(6);
  res_str << "  translation:\n";
  res_str << "    x: " << camera_imu_pose.t_x << "\n";
  res_str << "    y: " << camera_imu_pose.t_y << "\n";
  res_str << "    z: " << camera_imu_pose.t_z << "\n";
  res_str << "  quaternion:\n";
  res_str << "    x: " << camera_imu_pose.q_x << "\n";
  res_str << "    y: " << camera_imu_pose.q_y << "\n";
  res_str << "    z: " << camera_imu_pose.q_z << "\n";
  res_str << "    w: " << camera_imu_pose.q_w << "\n\n";

  auto lidar_imu_pose = robosense::calib::factory_calibration::getPoseFromMatrix(lidar_extrinsic_.transform_matrix);
  res_str << "lidar extrinsic:\n";
  res_str << std::fixed << std::setprecision(6);
  res_str << "  translation:\n";
  res_str << "    x: " << lidar_imu_pose.t_x << "\n";
  res_str << "    y: " << lidar_imu_pose.t_y << "\n";
  res_str << "    z: " << lidar_imu_pose.t_z << "\n";
  res_str << "  quaternion:\n";
  res_str << "    x: " << lidar_imu_pose.q_x << "\n";
  res_str << "    y: " << lidar_imu_pose.q_y << "\n";
  res_str << "    z: " << lidar_imu_pose.q_z << "\n";
  res_str << "    w: " << lidar_imu_pose.q_w << "\n\n";

  std::cout << std::endl;
  std::cout << res_str.str();
  return res_str.str();
}

std::string CalibCore::getCalibResult() {
  switch (calib_status_) {
  case CalibStatus::None: break;
  case CalibStatus::CameraInt: return camera_calib_ptr_->getCalibResult();
  case CalibStatus::Ext: return getCameraLidarCalibResult(); break;
  default: break;
  }
  return "";
}

bool CalibCore::isExtCalibReady() {
  return is_ext_calib_ready_;
}

void CalibCore::setExtCalibReady(const bool status) {
  is_ext_calib_ready_ = status;
}

void CalibCore::addExtCalibPose() {
  ext_calib_pose_count_++;
}

size_t CalibCore::getExtCalibPose() {
  return ext_calib_pose_count_;
}

void CalibCore::clearExtCalibPose() {
  ext_calib_pose_count_ = 0;
}

void CalibCore::solveHandEyeCalibration(const std::vector<Eigen::Matrix4d>& matrix_A,
                                        const std::vector<Eigen::Matrix4d>& matrix_B,
                                        Eigen::Matrix4d& _transform_X) {
  if (matrix_A.size() != matrix_B.size() || matrix_A.size() <= 3) {
    return;
  }
  // calculate R
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0, m_size = matrix_A.size(); i < m_size; ++i) {
    const Eigen::Matrix3d R_A = matrix_A[i].block<3, 3>(0, 0);
    const Eigen::Matrix3d R_B = matrix_B[i].block<3, 3>(0, 0);
    M += R_B * R_A.transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d R        = svd.matrixU() * svd.matrixV().transpose();
  _transform_X.block<3, 3>(0, 0) = R;

  // calculate t
  Eigen::MatrixXd C(3 * matrix_A.size(), 3);
  Eigen::VectorXd d(3 * matrix_A.size());

  const Eigen::Matrix3d c = Eigen::Matrix3d::Identity() - R;
  for (std::size_t i = 0, m_size = matrix_A.size(); i < m_size; ++i) {
    const Eigen::Vector3d t_A = matrix_A[i].block<3, 1>(0, 3);
    const Eigen::Vector3d t_B = matrix_B[i].block<3, 1>(0, 3);
    C.block<3, 3>(3 * i, 0)   = c;
    d.segment<3>(3 * i)       = t_B - R * t_A;
  }

  const Eigen::Matrix<double, 3, 1> t = C.colPivHouseholderQr().solve(d);
  _transform_X.block<3, 1>(0, 3)      = t;
  return;
}