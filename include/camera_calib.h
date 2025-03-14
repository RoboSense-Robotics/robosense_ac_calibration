#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <eigen3/Eigen/Core>
#include <iostream>
#include <stack>

#include "board_info.h"
#include "camera_model.h"
#include "camera_model_factory.h"
#include "corner_point_detector.h"
#include "lidar_calib.h"
// #  include "camera_model/camera_model_factory.h"

namespace camera_calib {
enum TagStyle {
  AprilTag   = 0,
  AprilTag2  = 1,
  ChessBoard = 2,
};

struct CalibResult {
  cv::Mat camera_matrix;
  cv::Mat distort_coefficient;
  double calib_error = 0.0;
};

class CameraCalib {
private:
  void loadCameraIntrinsics();

  bool getImageDetection(const cv::Mat& img_org,
                         cv::Mat& img_dst,
                         std::vector<robosense::calib::ChAprilBoardPoints>& detection,
                         const std::string& file,
                         const bool& _save_pic);

  bool getDetection(const cv::Mat& img_org, cv::Mat& img_dst, const std::string& file, const bool& _save_pic);
  bool autoDetection(const cv::Mat& img_org, cv::Mat& img_dst, const std::string& file, const bool& _save_pic);
  void addDetectionResult();
  bool isInsideValidArea(const cv::Point2d& point_2d);
  double calculateArea(const std::vector<cv::Point2d>& points);

  void writeDetectionResultInCSV();
  void readDetectionResultInCSV();
  void exportCalibResult();
  void checkCalib();
  void plotError();

  void debugCalibrate();
  void calibrateCameraIntrinsics();

  std::string april_tag_config_file_;
  std::string output_path_;
  std::string raw_data_path_;

  robosense::calib::ChAprilTagDetector detector_;
  TagStyle calib_case_;
  robosense::CameraModelType camera_model_type_ = robosense::CameraModelType::kWidePinhole;

  std::vector<cv::Mat> img_vec_;
  std::vector<std::string> file_names_;
  std::vector<std::vector<cv::Point3d>> base_points_;
  std::vector<std::vector<cv::Point2d>> pic_points_;
  std::vector<std::vector<int>> points_indices_;
  std::vector<std::vector<cv::Point3d>> corner_map_;
  cv::Size img_size_;
  std::vector<std::vector<double>> r_vecs_, t_vecs_;
  double calib_err_                                     = 10000.0;
  std::shared_ptr<robosense::CameraModel> camera_model_ = nullptr;

  std::stack<cv::Mat> img_stack_;
  std::vector<cv::Point2d> points_2d_valid_;
  std::vector<cv::Point3d> points_3d_valid_;
  std::vector<int> index_valid_;

  std::vector<cv::Point> indicate_polygon_;

  size_t current_pose_;
  size_t current_index_;
  CalibResult calib_res_;

  std::shared_ptr<lidar_calib::LidarCalib> lidar_calib_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_pointcloud_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_board_pointcloud_ptr_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fitted_board_pointcloud_ptr_;

  size_t pointcloud_count_ = 0;

  Eigen::Vector4f min_pt_;
  Eigen::Vector4f max_pt_;
  Eigen::Matrix4d lidar_board_pose_;

public:
  CameraCalib();
  ~CameraCalib();
  void loadConfig(const std::string& _config_file);

  void setCameraModel(const std ::string& camera_model,
                      const cv::Mat& camera_int_matrix,
                      const cv::Mat& camera_dist_coeff,
                      const cv::Size& camera_image_size);
  void calibCameraExtrinsics(const cv::Mat& img_org, Eigen::Matrix4d& camera_pose);

  // void getImageInput(const std::string& _data_path);
  void initDetector();
  void pushImage(const cv::Mat& img_org, cv::Mat& img_dst);
  void popImage();
  void startCalib();

  void addBoardIndicator(cv::Mat& img);
  bool autoPushImage(cv::Mat& img);

  size_t getImageCount();
  std::string getCalibResult();
};
}  // namespace camera_calib

#endif