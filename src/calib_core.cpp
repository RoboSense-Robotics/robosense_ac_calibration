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

Eigen::Matrix4d CalibCore::calibrateCamera2LidarExtrinsics() {
  std::vector<Eigen::Matrix4d> camera_pose_vec, lidar_pose_vec;
  for (size_t i = 0; i < static_pose_timestamp_vec_.size(); ++i) {
    cv::Mat image = cv_bridge::toCvCopy(image_msg_vec_[i], image_msg_vec_[i]->encoding)->image;

    auto cloud_ptr = pointcloud_vec_[i];
    Eigen::Matrix4d camera_pose, lidar_pose;
    calibrateSingleCameraLidarExtrinsics(image, cloud_ptr, camera_pose, lidar_pose);
    camera_pose_vec.push_back(camera_pose);
    lidar_pose_vec.push_back(lidar_pose);
  }
  auto camera_lidar_transform = calibrateCameraLidarExtrinsics(camera_pose_vec, lidar_pose_vec);
  return camera_lidar_transform;
}

Eigen::Matrix4d CalibCore::calibrateCamera2IMUExtrinsics() {
  std::cout << "static_pose_timestamp_vec: " << static_pose_timestamp_vec_.size() << std::endl;

  std::vector<Eigen::Matrix4d> camera_pose_vec, camera_delta_pose;
  for (size_t i = 0; i < static_pose_timestamp_vec_.size(); ++i) {
    cv::Mat image = cv_bridge::toCvCopy(image_msg_vec_[i], image_msg_vec_[i]->encoding)->image;
    Eigen::Matrix4d camera_pose;
    calibrateSingleCameraExtrinsics(image, camera_pose);

    if (!camera_pose_vec.empty()) {
      Eigen::Matrix4d delta_transform = (camera_pose.inverse() * camera_pose_vec.back()).inverse();
      camera_delta_pose.push_back(delta_transform);
      // auto delta_pose = robosense::calib::factory_calibration::getPoseFromMatrix(delta_transform);
      // std::cout << std::fixed << std::setprecision(6);
      // std::cout << "=================================================" << std::endl;
      // std::cout << "camera_delta_pose euler:\n";
      // std::cout << "    roll:  " << delta_pose.euler_vec(2) * 180.0 / M_PI << "\n";
      // std::cout << "    pitch: " << delta_pose.euler_vec(1) * 180.0 / M_PI << "\n";
      // std::cout << "    yaw:   " << delta_pose.euler_vec(0) * 180.0 / M_PI << "\n";
      // std::cout << "\n";
    }
    camera_pose_vec.push_back(camera_pose);
  }

  Eigen::Matrix4d camera_imu_transform = Eigen::Matrix4d::Identity();
  if (camera_delta_pose.size() < 3) {
    std::cout << "lack of data for hand-eye calibration" << std::endl;
    camera_imu_transform = camera_extrinsic_.transform_matrix;
  } else {
    std::vector<Eigen::Matrix4d> imu_delta_pose = integrateIMU(imu_msg_queue_, static_pose_timestamp_vec_);
    std::cout << "handeye calibration pose count: " << camera_delta_pose.size() << " -> " << imu_delta_pose.size()
              << std::endl;
    auto imu_camera_transform = calibrateCameraImuExtrinsics(camera_delta_pose, imu_delta_pose);
    camera_imu_transform      = imu_camera_transform.inverse();

    camera_imu_transform.block<3, 1>(0, 3) = camera_extrinsic_.transform_matrix.block<3, 1>(0, 3);  // keep translation
  }
  return camera_imu_transform;
}

void CalibCore::calibrateExtrinsics() {
  // load default extrinsic
  Eigen::Matrix4d camera_imu_transform   = camera_extrinsic_.transform_matrix;
  Eigen::Matrix4d lidar_imu_transform    = lidar_extrinsic_.transform_matrix;
  Eigen::Matrix4d camera_lidar_transform = lidar_imu_transform.inverse() * camera_imu_transform;

  if (calib_status_ == CalibStatus::Camera2Lidar) {
    camera_lidar_transform = calibrateCamera2LidarExtrinsics();
    camera_imu_transform   = lidar_imu_transform * camera_lidar_transform;

    camera_extrinsic_.transform_matrix = camera_imu_transform;
  } else if (calib_status_ == CalibStatus::Camera2IMU) {
    camera_imu_transform = calibrateCamera2IMUExtrinsics();
    lidar_imu_transform  = camera_imu_transform * camera_lidar_transform.inverse();

    camera_extrinsic_.transform_matrix = camera_imu_transform;
    lidar_extrinsic_.transform_matrix  = lidar_imu_transform;
  } else {
    std::cout << "calib_status_ error: " << calib_status_ << std::endl;
  }
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

  return lidar_pose_vec[0].inverse() * camera_pose_vec[0];
}

Eigen::Matrix4d CalibCore::calibrateCameraImuExtrinsics(const std::vector<Eigen::Matrix4d>& camera_delta_pose_vec,
                                                        const std::vector<Eigen::Matrix4d>& imu_delta_pose_vec) {
  Eigen::Matrix4d imu_camera_transform = Eigen::Matrix4d::Identity();
  solveHandEyeCalibration(camera_delta_pose_vec, imu_delta_pose_vec, imu_camera_transform);
  return imu_camera_transform;
}

Eigen::Matrix4d CalibCore::calIMUTransform(const sensor_msgs::msg::Imu::SharedPtr& pre_imu_msg,
                                           const sensor_msgs::msg::Imu::SharedPtr& nxt_imu_msg) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  double pre_time =
    static_cast<double>(pre_imu_msg->header.stamp.sec) + static_cast<double>(pre_imu_msg->header.stamp.nanosec) * 1e-9;
  double nxt_time =
    static_cast<double>(nxt_imu_msg->header.stamp.sec) + static_cast<double>(nxt_imu_msg->header.stamp.nanosec) * 1e-9;
  double delta_time = nxt_time - pre_time;

  double angular_velocity_x =
    (nxt_imu_msg->angular_velocity.x + pre_imu_msg->angular_velocity.x) / 2.0 - angular_velocity_bias_(0);
  double angular_velocity_y =
    (nxt_imu_msg->angular_velocity.y + pre_imu_msg->angular_velocity.y) / 2.0 - angular_velocity_bias_(1);
  double angular_velocity_z =
    (nxt_imu_msg->angular_velocity.z + pre_imu_msg->angular_velocity.z) / 2.0 - angular_velocity_bias_(2);

  double angular_x = angular_velocity_x * delta_time;
  double angular_y = angular_velocity_y * delta_time;
  double angular_z = angular_velocity_z * delta_time;
  if (delta_time > 1.0) {
    return transform;
  }

  // IMU angular represents the rotation axis
  Eigen::Vector3d rotation_vec;
  rotation_vec << angular_x, angular_y, angular_z;

  double angle         = rotation_vec.norm();
  Eigen::Vector3d axis = rotation_vec.normalized();

  Eigen::AngleAxisd angle_axis(angle, axis);

  Eigen::Matrix3d rotation_matrix = angle_axis.toRotationMatrix();

  transform.setIdentity();
  transform.block(0, 0, 3, 3) = rotation_matrix;

  return transform;
}

Eigen::Matrix4d CalibCore::integrateIMUPose(const std::queue<sensor_msgs::msg::Imu::SharedPtr>& imu_queue) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  auto imu_queue_copy   = imu_queue;
  auto pre_imu_msg      = imu_queue_copy.front();
  size_t interval_count = 0;

  while (1) {
    imu_queue_copy.pop();
    interval_count++;
    if (imu_queue_copy.empty()) {
      break;
    }
    interval_count = 0;

    auto nxt_imu_msg     = imu_queue_copy.front();
    auto delta_transform = calIMUTransform(pre_imu_msg, nxt_imu_msg);

    auto imu_pose = robosense::calib::factory_calibration::getPoseFromMatrix(delta_transform);

    Eigen::Matrix3d rotation_matrix = delta_transform.block<3, 3>(0, 0);
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    double rotation_angle = rotation_vector.angle() * 180.0 / M_PI;
    if (rotation_angle > 20) {
      pre_imu_msg = nxt_imu_msg;
      continue;
    }

    transform   = transform * delta_transform;
    pre_imu_msg = nxt_imu_msg;
  }
  return transform;
}

std::vector<Eigen::Matrix4d> CalibCore::integrateIMU(const std::queue<sensor_msgs::msg::Imu::SharedPtr>& imu_queue,
                                                     const std::vector<double>& static_pose_timestamp_vec) {
  std::cout << "integrateIMU: " << imu_queue.size() << " -> " << static_pose_timestamp_vec.size() << std::endl;

  std::vector<Eigen::Matrix4d> delta_imu_transform_vec;
  if (imu_queue.size() < 2 || static_pose_timestamp_vec.size() < 2) {
    return delta_imu_transform_vec;
  }

  auto imu_queue_copy = imu_queue;

  for (size_t i = 0; i < static_pose_timestamp_vec.size() - 1; ++i) {
    auto start_timestamp = static_pose_timestamp_vec[i];
    auto ent_timestamp   = static_pose_timestamp_vec[i + 1];
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_segment_queue;

    while (!imu_queue_copy.empty()) {
      auto imu_msg = imu_queue_copy.front();
      double current_stamp =
        static_cast<double>(imu_msg->header.stamp.sec) + static_cast<double>(imu_msg->header.stamp.nanosec) * 1e-9;

      if (current_stamp < start_timestamp) {
        imu_queue_copy.pop();
        continue;
      }

      if (current_stamp > ent_timestamp) {

        // std::cout << "**************************************" << std::endl;
        std::cout << "timestamp gap: " << start_timestamp << " -> " << ent_timestamp << std::endl;
        auto imu_transform = integrateIMUPose(imu_segment_queue);

        auto imu_pose = robosense::calib::factory_calibration::getPoseFromMatrix(imu_transform);
        // std::cout << "imu_delta_pose euler:\n";
        // std::cout << std::fixed << std::setprecision(6);
        // std::cout << "    roll:  " << imu_pose.euler_vec(2) * 180.0 / M_PI << "\n";
        // std::cout << "    pitch: " << imu_pose.euler_vec(1) * 180.0 / M_PI << "\n";
        // std::cout << "    yaw:   " << imu_pose.euler_vec(0) * 180.0 / M_PI << "\n";
        // std::cout << "\n";

        imu_segment_queue = std::queue<sensor_msgs::msg::Imu::SharedPtr>();
        delta_imu_transform_vec.push_back(imu_transform);
        break;
      }

      imu_segment_queue.push(imu_msg);
      imu_queue_copy.pop();
    }
  }
  // exit(1);

  return delta_imu_transform_vec;
}

void CalibCore::startCalib(const CalibStatus calib_status) {
  switch (calib_status) {
  case CalibStatus::None: break;
  case CalibStatus::CameraInt: calibrateCameraIntrinsics(); break;
  case CalibStatus::Camera2Lidar: calibrateExtrinsics(); break;
  case CalibStatus::Camera2IMU: calibrateExtrinsics(); break;
  default: break;
  }
}

bool CalibCore::autoPushImage(cv::Mat& img) {
  return camera_calib_ptr_->autoPushImage(img);
}

size_t CalibCore::getImageCount() {
  return camera_calib_ptr_->getImageCount();
}
void CalibCore::setCalibStatus(const CalibStatus calib_status){
  calib_status_ = calib_status;
}

void CalibCore::setPointcloudInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& _input_ptr) {
  pointcloud_vec_.push_back(_input_ptr);
}

void CalibCore::setImuMsgInput(const sensor_msgs::msg::Imu::SharedPtr& msg) {
  imu_msg_queue_.push(msg);

  double angular_thresh = 0.1;  // unit: rad/s
  double current_stamp =
    static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  const bool is_moving = checkMotion(msg, angular_thresh);
  last_motion_time_    = last_motion_time_ < 1e-3 ? current_stamp : last_motion_time_;

  bias_collected_ = (calib_status_ == CalibStatus::Camera2IMU) ? bias_collected_ : true;

  if (is_moving) {
    last_motion_time_ = current_stamp;
    silence_detected_ = false;
  } else {
    double silence_duration = current_stamp - last_motion_time_;

    if (!bias_collected_ && silence_duration >= 10.0) {
      // need 10s silence to calibrate imu bias
      calibrateIMUBias();
    }

    if (bias_collected_ && silence_duration >= 2.0 && !silence_detected_) {
      silence_detected_ = true;
      // static_pose_timestamp_vec_.push_back(current_stamp);
      std::cout << "static_pose_timestamp: " << std::fixed << std::setprecision(3) << current_stamp
                << "      imu_msg_queue size: " << imu_msg_queue_.size() << std::endl;
      is_ext_calib_ready_ = true;
    }
  }
  return;
}

bool CalibCore::calibrateIMUBias() {
  bias_collected_         = true;
  auto imu_msg_queue_copy = imu_msg_queue_;
  angular_velocity_bias_.setZero();
  size_t imu_msg_count = 0;
  while (!imu_msg_queue_copy.empty()) {
    auto msg = imu_msg_queue_copy.front();
    Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    angular_velocity_bias_ += angular_velocity;
    imu_msg_queue_copy.pop();
    imu_msg_count++;
  }
  if (imu_msg_count < 1) {
    angular_velocity_bias_.setZero();
    return false;
  }
  angular_velocity_bias_ /= imu_msg_count;
  std::cout << "angular_velocity_bias has been calibrated!" << std::endl;
  return true;
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
  res_str << "    w: " << camera_imu_pose.q_w << "\n";

  res_str << "  euler:\n";
  res_str << "    roll:  " << camera_imu_pose.euler_vec(2) * 180.0 / M_PI << "\n";
  res_str << "    pitch: " << camera_imu_pose.euler_vec(1) * 180.0 / M_PI << "\n";
  res_str << "    yaw:   " << camera_imu_pose.euler_vec(0) * 180.0 / M_PI << "\n";
  res_str << "\n";

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
  res_str << "    w: " << lidar_imu_pose.q_w << "\n";
  res_str << "  euler:\n";
  res_str << "    roll:  " << lidar_imu_pose.euler_vec(2) * 180.0 / M_PI << "\n";
  res_str << "    pitch: " << lidar_imu_pose.euler_vec(1) * 180.0 / M_PI << "\n";
  res_str << "    yaw:   " << lidar_imu_pose.euler_vec(0) * 180.0 / M_PI << "\n";
  res_str << "\n";

  std::cout << std::endl;
  std::cout << res_str.str();
  return res_str.str();
}

std::string CalibCore::getCalibResult() {
  switch (calib_status_) {
  case CalibStatus::None: break;
  case CalibStatus::CameraInt: return camera_calib_ptr_->getCalibResult();
  case CalibStatus::Camera2Lidar: return getCameraLidarCalibResult(); break;
  case CalibStatus::Camera2IMU: return getCameraLidarCalibResult(); break;
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

Eigen::Matrix3d solveHandEyeRotation(const std::vector<Eigen::Matrix4d>& matrix_A,
                                     const std::vector<Eigen::Matrix4d>& matrix_B) {
  // calculate R
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0, m_size = matrix_A.size(); i < m_size; ++i) {
    const Eigen::Matrix3d R_A = matrix_A[i].block<3, 3>(0, 0);
    const Eigen::Matrix3d R_B = matrix_B[i].block<3, 3>(0, 0);
    Eigen::AngleAxis<double> angleAxis_A(R_A);
    Eigen::AngleAxis<double> angleAxis_B(R_B);
    double angle_A = angleAxis_A.angle() * 180.0 / M_PI;
    double angle_B = angleAxis_B.angle() * 180.0 / M_PI;

    // if (std::fabs(angle_A - angle_B) > 10) {
    //   continue;
    // }
    // std::cout << "pose " << i << "    " << angle_A << ",    \t" << angle_B << std::endl;

    Eigen::Matrix<double, 3, 1> rotationVector_A = angleAxis_A.angle() * angleAxis_A.axis();
    Eigen::Matrix<double, 3, 1> rotationVector_B = angleAxis_B.angle() * angleAxis_B.axis();
    M += rotationVector_B * rotationVector_A.transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_X = svd.matrixV() * svd.matrixU().transpose();
  return R_X;
}

void CalibCore::solveHandEyeCalibration(const std::vector<Eigen::Matrix4d>& matrix_A,
                                        const std::vector<Eigen::Matrix4d>& matrix_B,
                                        Eigen::Matrix4d& _transform_X) {
  // A*X = X*B
  if (matrix_A.size() != matrix_B.size() || matrix_A.size() < 2) {
    return;
  }
  Eigen::Matrix3d R_X = solveHandEyeRotation(matrix_A, matrix_B);
  _transform_X.setIdentity();
  _transform_X.block<3, 3>(0, 0) = R_X;
  return;
}