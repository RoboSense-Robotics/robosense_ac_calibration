#include "calib_core.h"
#include "common_function.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

using namespace camera_calib;

CameraCalib::CameraCalib() {
}

CameraCalib::~CameraCalib() {
}

void CameraCalib::loadCameraIntrinsics() {
}

bool CameraCalib::getImageDetection(const cv::Mat& img_org,
                                    cv::Mat& img_dst,
                                    std::vector<robosense::calib::ChAprilBoardPoints>& detection,
                                    const std::string& file,
                                    const bool& _save_pic) {
  img_size_ = img_org.size();
  detector_.detect(img_org, img_dst);
  detection = detector_.getChAprilBoardPoints();
  if (detection.empty()) {
    return false;
  }

  if (_save_pic) {
    std::string output_pic_path_ = output_path_ + "/detection_result_pic/";
    mkdir(output_pic_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cv::imwrite(output_pic_path_ + "detect_" + file, img_dst);
  }
  return true;
}

bool CameraCalib::getDetection(const cv::Mat& img_org,
                               cv::Mat& img_dst,
                               const std::string& file,
                               const bool& _save_pic) {

  img_size_ = img_org.size();
  detector_.detect(img_org, img_dst);
  auto detection = detector_.getChAprilBoardPoints();
  // detector_.printChAprilBoardPoints();

  if (detection.empty()) {
    return false;
  }

  if (_save_pic) {
    std::string output_pic_path_ = output_path_ + "/detection_result_pic/";
    mkdir(output_pic_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cv::imwrite(output_pic_path_ + "detect_" + file, img_dst);
  }

  std::vector<cv::Point2d> points_2d_valid;
  std::vector<cv::Point3d> points_3d_valid;
  std::vector<int> index_valid;
  int image_size_height   = img_size_.height;
  int image_size_width    = img_size_.width;
  double radius_threshold = (image_size_height + image_size_width) * (image_size_height + image_size_width) / 16;
  if (static_cast<int>(camera_model_type_) != 2) {
    radius_threshold *= 16;
  }

  auto points_2d      = detection[0].chess_corners_2d;
  auto grid_points_3d = detection[0].chess_corners_3d;
  auto index_vec      = detection[0].chess_corner_indices;

  for (std::size_t i = 0; i < points_2d.size(); ++i) {
    double radius = (points_2d[i].x - image_size_width / 2) * (points_2d[i].x - image_size_width / 2) +
                    (points_2d[i].y - image_size_height / 2) * (points_2d[i].y - image_size_height / 2);

    points_3d_valid.push_back(static_cast<cv::Point3d>(grid_points_3d[i]));
    points_2d_valid.push_back(static_cast<cv::Point2d>(points_2d[i]));
    index_valid.push_back(index_vec[i]);
  }

  if (points_2d_valid.size() < 1) {
    return false;
  }
  std::cout << "2d points size: " << points_2d_valid.size() << std::endl;
  std::cout << "3d points size: " << points_3d_valid.size() << std::endl << std::endl;

  base_points_.push_back(points_3d_valid);
  pic_points_.push_back(points_2d_valid);
  points_indices_.push_back(index_valid);
  return true;
}

bool CameraCalib::autoDetection(const cv::Mat& img_org,
                                cv::Mat& img_dst,
                                const std::string& file,
                                const bool& _save_pic) {

  img_size_ = img_org.size();
  detector_.detect(img_org, img_dst);
  auto detection = detector_.getChAprilBoardPoints();
  // detector_.printChAprilBoardPoints();

  if (detection.empty()) {
    return false;
  }

  if (_save_pic) {
    std::string output_pic_path_ = output_path_ + "/detection_result_pic/";
    mkdir(output_pic_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cv::imwrite(output_pic_path_ + "detect_" + file, img_dst);
  }

  points_2d_valid_.clear();
  points_3d_valid_.clear();
  index_valid_.clear();

  int image_size_height   = img_size_.height;
  int image_size_width    = img_size_.width;
  double radius_threshold = (image_size_height + image_size_width) * (image_size_height + image_size_width) / 16;
  if (static_cast<int>(camera_model_type_) != 2) {
    radius_threshold *= 16;
  }

  auto points_2d      = detection[0].chess_corners_2d;
  auto grid_points_3d = detection[0].chess_corners_3d;
  auto index_vec      = detection[0].chess_corner_indices;

  for (std::size_t i = 0; i < points_2d.size(); ++i) {
    double radius = (points_2d[i].x - image_size_width / 2) * (points_2d[i].x - image_size_width / 2) +
                    (points_2d[i].y - image_size_height / 2) * (points_2d[i].y - image_size_height / 2);

    points_3d_valid_.push_back(static_cast<cv::Point3d>(grid_points_3d[i]));
    points_2d_valid_.push_back(static_cast<cv::Point2d>(points_2d[i]));
    index_valid_.push_back(index_vec[i]);
  }

  if (points_2d_valid_.size() < 1) {
    return false;
  }

  for (const auto& point_2d : points_2d_valid_) {
    if (!isInsideValidArea(point_2d)) {
      return false;
    }
  }

  double area = calculateArea(points_2d_valid_);
  area *= 25 / 9;  // 5*5 -> 3*3
  double area_ratio = area / img_size_.area() * 100;
  std::cout << "area is too small: " << area << "      \t" << area_ratio << "%" << std::endl;
  if (area_ratio < 8.0) {

    return false;
  }

  std::cout << "2d points size: " << points_2d_valid_.size() << std::endl;
  std::cout << "3d points size: " << points_3d_valid_.size() << std::endl << std::endl;
  addDetectionResult();
  return true;
}

bool CameraCalib::isInsideValidArea(const cv::Point2d& point_2d) {
  cv::Point2d A = indicate_polygon_[0];
  cv::Point2d B = indicate_polygon_[1];
  cv::Point2d C = indicate_polygon_[2];
  cv::Point2d D = indicate_polygon_[3];
  cv::Point2d P = point_2d;

  auto cross = [](const cv::Point2d& p1, const cv::Point2d& p2) { return p1.x * p2.y - p1.y * p2.x; };

  // 计算每条边的叉积，确保所有叉积结果符号一致（同侧）
  double cross1 = cross(B - A, P - A);
  double cross2 = cross(C - B, P - B);
  double cross3 = cross(D - C, P - C);
  double cross4 = cross(A - D, P - D);

  // 判断是否都在同一侧
  return (cross1 >= 0 && cross2 >= 0 && cross3 >= 0 && cross4 >= 0) ||
         (cross1 <= 0 && cross2 <= 0 && cross3 <= 0 && cross4 <= 0);
}

double CameraCalib::calculateArea(const std::vector<cv::Point2d>& points) {
  if (points.size() < 3) {
    return 0.0;  // 小于三个点无法形成多边形
  }

  // 将cv::Point2d转为CV_32F类型的cv::Mat
  std::vector<cv::Point2f> points_f;
  for (const auto& point : points) {
    points_f.push_back(cv::Point2f(point.x, point.y));
  }

  // 计算凸包
  std::vector<cv::Point2f> hull;
  cv::convexHull(points_f, hull);
  return cv::contourArea(hull);
}

void CameraCalib::addDetectionResult() {
  base_points_.push_back(points_3d_valid_);
  pic_points_.push_back(points_2d_valid_);
  points_indices_.push_back(index_valid_);
}

void CameraCalib::writeDetectionResultInCSV() {
  for (size_t i = 0; i < base_points_.size(); i++) {
    auto point_3d  = base_points_[i];
    auto point_2d  = pic_points_[i];
    auto index_vec = points_indices_[i];

    std::ofstream out_file(output_path_ + "/detection_result_csv/" + std::to_string(i) + "_detect_result.csv",
                           std::ios_base::out);

    if (index_vec.size() == 0) {
      continue;
    }
    for (int index = 0; index < index_vec.size(); index++) {
      out_file << i << "," << index_vec[index] << ",";
      out_file << point_2d[index].x << "," << point_2d[index].y << ",";
      out_file << point_3d[index].x << "," << point_3d[index].y << "," << point_3d[index].z << "\n";
    }
    out_file.close();
  }
  std::cout << "Detection result has been writen down: " << output_path_ + "/detection_result_csv." << std::endl;
  return;
}

void CameraCalib::readDetectionResultInCSV() {
}

void CameraCalib::loadConfig(const std::string& _config_file) {
  if (access(_config_file.c_str(), F_OK) == -1) {
    std::cout << "Camera config file does not exist!!!      " << _config_file << std::endl;
    return;
  }
  std::cout << "Camera config file: " << _config_file << std::endl;
  YAML::Node doc = YAML::LoadFile(_config_file);

  output_path_ = doc["output_dir"].as<std::string>();
  std::cout << "    output_dir: " << output_path_ << std::endl;

  mkdir((output_path_).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir((output_path_ + "/detection_result_csv").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  calib_case_ = AprilTag;
  if (doc["BOARD"]["pattern"].as<std::string>() == "AprilTag") {
    calib_case_ = AprilTag;
  }
  switch (calib_case_) {
  case AprilTag: april_tag_config_file_ = doc["BOARD"]["AprilTagConfig"].as<std::string>(); break;
  case AprilTag2: break;
  case ChessBoard: break;
  default: break;
  }
  std::cout << "    INPUT_TYPE: " << doc["BOARD"]["pattern"].as<std::string>() << std::endl;
  std::cout << "    Tag_Config: " << april_tag_config_file_ << std::endl;

  robosense::CameraModelFactory camera_model_fac;
  std::string camera_type = doc["CAMERA"]["distort_model"].as<std::string>();

  if (camera_type == "Pinhole") {
    camera_type = doc["CAMERA"]["distort_coeff"].as<int>() == 5 ? "NormalPinhole" : "WidePinhole";
  }

  std::unordered_map<std::string, robosense::CameraModelType> camera_type_map = {
    { "NormalPinhole", robosense::CameraModelType::kNormalPinhole },
    { "WidePinhole", robosense::CameraModelType::kWidePinhole },
    { "Fisheye", robosense::CameraModelType::kFisheye }
  };

  camera_model_type_ = camera_type_map[camera_type];
  std::cout << "    CAMERA_TYPE: " << camera_type << "  " << static_cast<int>(camera_model_type_) << std::endl;
  if (camera_model_ == nullptr) {
    camera_model_ = camera_model_fac.GetCameraModel(camera_model_type_);
  }
  std::cout << "=============================================================" << std::endl;
  return;
}

void CameraCalib::setCameraModel(const std ::string& camera_model_type,
                                 const cv::Mat& camera_int_matrix,
                                 const cv::Mat& camera_dist_coeff,
                                 const cv::Size& camera_image_size) {

  std::unordered_map<std::string, robosense::CameraModelType> camera_type_map = {
    { "Pinhole", robosense::CameraModelType::kWidePinhole },
    { "Fisheye", robosense::CameraModelType::kFisheye }
  };

  auto camera_type = camera_type_map[camera_model_type];
  robosense::CameraModelFactory camera_model_fac;

  std::cout << "camera_matrix_:\n" << camera_int_matrix << std::endl;
  std::cout << "camera_dist_coeff:\n" << camera_dist_coeff << std::endl;
  camera_model_ = camera_model_fac.GetCameraModel(camera_int_matrix, camera_dist_coeff, camera_image_size, camera_type);
}

void CameraCalib::calibCameraExtrinsics(const cv::Mat& img_org, Eigen::Matrix4d& camera_pose) {
  std::vector<double> rvec, tvec;
  std::vector<cv::Point3d> base_points;
  std::vector<cv::Point2d> pic_points;
  cv::Mat img_dst;
  std::vector<robosense::calib::ChAprilBoardPoints> detection;
  cv::imwrite("/home/sti/0_super_sensor_sdks/open_source_sdks/src/calibration/debug/image.jpeg", img_org);
  if (getImageDetection(img_org, img_dst, detection, "", false)) {
    camera_model_->SolvePnP(detection[0].chess_corners_3d, detection[0].chess_corners_2d, rvec, tvec);
  }
  std::vector<double> camera_pose_vec = rvec;
  camera_pose_vec.insert(camera_pose_vec.end(), tvec.begin(), tvec.end());
  camera_pose = robosense::calib::factory_calibration::getMatrixFromRotationPose(camera_pose_vec);
  camera_pose = camera_pose.inverse();
}

void transformCameraPose(const std::vector<double>& board_to_camera_r_vec,
                         const std::vector<double>& board_to_camera_t_vec,
                         std::vector<double>& camera_to_board_r_vec,
                         std::vector<double>& camera_to_board_t_vec) {
  std::vector<double> board_to_camera_pose = board_to_camera_r_vec;
  board_to_camera_pose.insert(board_to_camera_pose.end(), board_to_camera_t_vec.begin(), board_to_camera_t_vec.end());

  auto board_to_camera_transform =
    robosense::calib::factory_calibration::getMatrixFromRotationPose(board_to_camera_pose);
  auto camera_to_board_transform = board_to_camera_transform.inverse();
  auto camera_to_board_pose = robosense::calib::factory_calibration::getEulerPoseFromMatrix(camera_to_board_transform);
  camera_to_board_r_vec     = std::vector<double>(camera_to_board_pose.begin(), camera_to_board_pose.begin() + 3);
  camera_to_board_t_vec     = std::vector<double>(camera_to_board_pose.begin() + 3, camera_to_board_pose.end());
}

void CameraCalib::debugCalibrate() {
  std::string _data_path = "/home/sti/4_data/0_camera_intrincs_data/20241211/H110S-F06020507";
  std::cout << "get raw image input: " << _data_path << std::endl;

  std::vector<std::string> file_names;
  auto file_num = robosense::calib::factory_calibration::getFileInFolder(_data_path.c_str(), "", file_names);
  std::cout << file_num << " files in total" << std::endl;
  int index = 0;
  for (auto file : file_names) {
    size_t dot_position        = file.find_last_of('.');
    std::string file_extension = file.substr(dot_position + 1);
    if (file_extension != "jpg" && file_extension != "png" && file_extension != "jpeg") {
      continue;
    }
    std::cout << "pic " << index << ":  " << file << std::endl;
    cv::Mat img_org = cv::imread(_data_path + "/" + file, cv::IMREAD_COLOR);
    cv::Mat img_dst;
    autoDetection(img_org, img_dst, "", false);
  }
}

void CameraCalib::calibrateCameraIntrinsics() {
  // debugCalibrate();
  std::vector<double> projection_err_vec;
  r_vecs_.clear();
  t_vecs_.clear();
  calib_err_ =
    camera_model_->CalibrateCamera(base_points_, pic_points_, img_size_, r_vecs_, t_vecs_, projection_err_vec);

  double x_min = 1000.0, y_min = 1000.0, z_min = 1000.0;
  double x_max = -1000.0, y_max = -1000.0, z_max = -1000.0;

  for (size_t i = 0; i < projection_err_vec.size(); i++) {
    std::vector<double> camera_to_board_r_vec, camera_to_board_t_vec;
    transformCameraPose(r_vecs_[i], t_vecs_[i], camera_to_board_r_vec, camera_to_board_t_vec);
    // std::cout << "pic " << i << ":    \t" << std::setprecision(12) << camera_to_board_t_vec[0] << ",    \t"
    //           << camera_to_board_t_vec[1] << ",    \t" << camera_to_board_t_vec[2] << std::endl;
    x_max = camera_to_board_t_vec[0] > x_max ? camera_to_board_t_vec[0] : x_max;
    y_max = camera_to_board_t_vec[1] > y_max ? camera_to_board_t_vec[1] : y_max;
    z_max = camera_to_board_t_vec[2] > z_max ? camera_to_board_t_vec[2] : z_max;
    x_min = camera_to_board_t_vec[0] < x_min ? camera_to_board_t_vec[0] : x_min;
    y_min = camera_to_board_t_vec[1] < y_min ? camera_to_board_t_vec[1] : y_min;
    z_min = camera_to_board_t_vec[2] < z_min ? camera_to_board_t_vec[2] : z_min;
  }

  std::cout << "\nCalibration err:" << std::endl
            << calib_err_ << std::endl
            << "\nCamera_matrix:" << std::endl
            << camera_model_->camera_matrix() << std::endl
            << "\nDistort_coefficient:" << std::endl
            << camera_model_->distort_coefficient() << std::endl;

  calib_res_.calib_error         = calib_err_;
  calib_res_.camera_matrix       = camera_model_->camera_matrix();
  calib_res_.distort_coefficient = camera_model_->distort_coefficient();

  exportCalibResult();
  checkCalib();
}

void CameraCalib::startCalib() {
  calibrateCameraIntrinsics();

  img_stack_     = std::stack<cv::Mat>();
  current_pose_  = img_stack_.size() / 6;
  current_index_ = img_stack_.size() % 6;
  return;
}

void CameraCalib::exportCalibResult() {
  std::vector<double> k_vec, d_vec;

  auto k = camera_model_->camera_matrix();
  std::ofstream file_k(output_path_ + "/K.csv", std::ios_base::out);
  for (int i = 0; i < k.rows; i++) {
    for (int j = 0; j < k.cols; j++) {
      file_k << k.at<double>(i, j);
      k_vec.push_back(k.at<double>(i, j));
      if (j < k.cols - 1) {
        file_k << ",";
      }
    }
    file_k << "\n";
  }
  file_k.close();

  auto d = camera_model_->distort_coefficient();
  std::ofstream file_d(output_path_ + "/D.csv", std::ios_base::out);
  for (int i = 0; i < d.rows; i++) {
    for (int j = 0; j < d.cols; j++) {
      file_d << d.at<double>(i, j);
      d_vec.push_back(d.at<double>(i, j));
      if (j < d.cols - 1) {
        file_d << ",";
      }
    }
    file_d << "\n";
  }
  file_d.close();

  std::ofstream file_yaml(output_path_ + "/intrinsics.yaml", std::ios_base::out);
  YAML::Node camera_info;
  camera_info["D"]            = d_vec;
  camera_info["K"]            = k_vec;
  camera_info["Mean Error"]   = calib_err_;
  camera_info["Camera model"] = static_cast<int>(camera_model_type_) > 1 ? "Fisheye" : "Pinhole";
  file_yaml << camera_info;
  file_yaml.close();
}

void CameraCalib::checkCalib() {
  std::vector<std::vector<cv::Point2d>> projected_points_vec;
  std::ofstream camera_out_file(output_path_ + "/projection.csv", std::ios_base::out);
  camera_out_file << "pic_id"
                  << ","
                  << "x"
                  << ","
                  << "y"
                  << ","
                  << "err"
                  << "\n";
  for (size_t i = 0; i < points_indices_.size(); i++) {
    std::vector<cv::Point2d> projected_points;
    camera_model_->ProjectPoints(base_points_[i], r_vecs_[i], t_vecs_[i], projected_points);

    for (size_t j = 0; j < projected_points.size(); j++) {
      double err = std::sqrt(std::pow(projected_points[j].x - pic_points_[i][j].x, 2) +
                             std::pow(projected_points[j].y - pic_points_[i][j].y, 2));
      camera_out_file << i << "," << pic_points_[i][j].x << "," << pic_points_[i][j].y << "," << err << "\n";
    }
  }
  camera_out_file.close();
}

void CameraCalib::plotError() {
  std::cout << "\nploting..." << std::endl;
  cv::Size image_size = camera_model_->image_size();

  std::ifstream projection_file(output_path_ + "/projection.csv", std::ios_base::in);
  std::string cur_str;
  std::vector<int> board_index_vec;
  std::vector<cv::Point2d> pic_points;
  std::vector<double> projection_err_vec;

  int line = 0;
  while (std::getline(projection_file, cur_str)) {
    line++;
    if (line < 2) {
      continue;
    }
    std::vector<double> cur_info;
    std::stringstream ss(cur_str);
    std::string str;
    while (getline(ss, str, ',')) {
      cur_info.push_back(std::atof(str.c_str()));
    }
    if (cur_info.size() < 4) {
      std::cerr << "error input in line " << line << ": " << cur_str << std::endl;
    }
    board_index_vec.push_back(int(cur_info[0]));
    pic_points.push_back(cv::Point2d(cur_info[1], cur_info[2]));
    projection_err_vec.push_back(double(cur_info[3]));
  }
  projection_file.close();

  int height = image_size.height;
  int width  = image_size.width;

  cv::Mat image(height, width, CV_64FC3, cv::Scalar(255, 255, 255));
  int line_width = 5;
  cv::line(image, cv::Point2d(0, 0), cv::Point2d(0, height), cv::Scalar(0, 0, 0), line_width);
  cv::line(image, cv::Point2d(width, height), cv::Point2d(0, height), cv::Scalar(0, 0, 0), line_width);
  cv::line(image, cv::Point2d(width, height), cv::Point2d(width, 0), cv::Scalar(0, 0, 0), line_width);
  cv::line(image, cv::Point2d(0, 0), cv::Point2d(width, 0), cv::Scalar(0, 0, 0), line_width);

  for (size_t i = 0; i < pic_points.size(); i++) {
    auto c = robosense::calib::factory_calibration::getColour(board_index_vec[i], board_index_vec.front(),
                                                              board_index_vec.back());
    cv::circle(image, pic_points[i], projection_err_vec[i] * 5, cv::Scalar(c[0] * 255, c[1] * 255, c[2] * 255), -1);
  }

  cv::Mat image_extend(height + 150, width + 600, CV_64FC3, cv::Scalar(255, 255, 255));
  cv::Rect roi(cv::Point(50, 100), image.size());
  image.copyTo(image_extend(roi));
  auto max_error_it = std::max_element(projection_err_vec.begin(), projection_err_vec.end());
  int max_err       = round(*max_error_it);
  double font_size  = 1.5;
  for (size_t i = 1; i < max_err; i++) {
    cv::circle(image_extend, cv::Point(width + 130, height - i * 100 + 100), (i * 5), cv::Scalar(0, 0, 0), -1);
    cv::putText(image_extend, std::to_string(i), cv::Point(width + 260, height - i * 100 + 120),
                cv::FONT_HERSHEY_TRIPLEX, font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  }
  cv::putText(image_extend, "reprojetion_error", cv::Point(width / 2, 70), cv::FONT_HERSHEY_TRIPLEX, font_size,
              cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  cv::putText(image_extend, "size", cv::Point(width + 90, height - max_err * 100 + 120), cv::FONT_HERSHEY_TRIPLEX,
              font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  cv::putText(image_extend, "error", cv::Point(width + 210, height - max_err * 100 + 120), cv::FONT_HERSHEY_TRIPLEX,
              font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);

  font_size = 1.3;
  cv::putText(image_extend, "max_error = " + std::to_string(*max_error_it), cv::Point(width + 80, 125),
              cv::FONT_HERSHEY_TRIPLEX, font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  cv::putText(image_extend, "avg_error = " + std::to_string(calib_err_).substr(0, 5), cv::Point(width + 80, 175),
              cv::FONT_HERSHEY_TRIPLEX, font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  cv::putText(image_extend, "pic_count = " + std::to_string(board_index_vec.back()), cv::Point(width + 80, 225),
              cv::FONT_HERSHEY_TRIPLEX, font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);
  cv::putText(image_extend, "corner_count = " + std::to_string(line - 1), cv::Point(width + 80, 275),
              cv::FONT_HERSHEY_TRIPLEX, font_size, cv::Scalar(0, 0, 0), (2), cv::LINE_AA);

  cv::imwrite(output_path_ + "/projection.jpeg", image_extend);
}

void CameraCalib::pushImage(const cv::Mat& img_org, cv::Mat& img_dst) {

  if (getDetection(img_org, img_dst, "", false)) {
    img_stack_.push(img_org);
    std::cout << "img_stack size: " << img_stack_.size() << std::endl;
  } else {
    std::cout << "getDetection failed" << std::endl;
  }
  current_pose_  = img_stack_.size() / 6;
  current_index_ = img_stack_.size() % 6;
}

bool CameraCalib::autoPushImage(cv::Mat& img) {
  cv::Mat img_dst;
  if (autoDetection(img, img_dst, "", false)) {
    img_stack_.push(img);
    std::cout << "img_stack size: " << img_stack_.size() << std::endl;
    current_pose_  = img_stack_.size() / 6;
    current_index_ = img_stack_.size() % 6;
    img            = img_dst;
    return true;
  }
  current_pose_  = img_stack_.size() / 6;
  current_index_ = img_stack_.size() % 6;
  return false;
}

void CameraCalib::popImage() {
  if (img_stack_.empty()) {
    std::cout << "no images..." << std::endl;
    return;
  }
  img_stack_.pop();
  current_pose_  = img_stack_.size() / 6;
  current_index_ = img_stack_.size() % 6;
}

void CameraCalib::initDetector() {
  detector_.init(april_tag_config_file_);
  detector_.setSubPixelWindowSize(cv::Size(7, 7));
}

size_t CameraCalib::getImageCount() {
  return img_stack_.size();
}

std::string formatMatrix(const cv::Mat& mat) {
  std::stringstream ss;
  for (int i = 0; i < mat.rows; i++) {
    for (int j = 0; j < mat.cols; j++) {
      ss << std::fixed << std::setprecision(6) << mat.at<double>(i, j);
      if (j < mat.cols - 1) {
        ss << "\n";
      }
    }
    ss << "\n";
  }
  return ss.str();
}

std::string CameraCalib::getCalibResult() {
  std::stringstream res_str;
  
  res_str << "calib_error:\n";
  res_str << std::fixed << std::setprecision(6) << calib_res_.calib_error << "\n\n";

  res_str << "fx: "<<calib_res_.camera_matrix.at<double>(0, 0) << "\n";
  res_str << "fy: "<<calib_res_.camera_matrix.at<double>(1, 1) << "\n";
  res_str << "cx: "<<calib_res_.camera_matrix.at<double>(0, 2) << "\n";
  res_str << "cy: "<<calib_res_.camera_matrix.at<double>(1, 2) << "\n\n";

  res_str << "distort_coefficient:\n";
  res_str << formatMatrix(calib_res_.distort_coefficient);

  return res_str.str();
}

void CameraCalib::addBoardIndicator(cv::Mat& img) {
  auto height = img.size().height;
  auto width  = img.size().width;

  double height_ratio = 0.6;
  double width_ratio  = 0.4;

  double width_interval_ratio = (1 - width_ratio) / 2;

  double start_y_ratio =
    (current_index_ <= 2) ? current_index_ * width_interval_ratio : (5 - current_index_) * width_interval_ratio;
  double start_x_ratio = (current_index_ <= 2) ? 0.0 : 1 - height_ratio;

  cv::Point point_lu(width * (start_y_ratio + 0.0), height * (start_x_ratio + 0.0));
  cv::Point point_ru(width * (start_y_ratio + 0.0), height * (start_x_ratio + height_ratio));
  cv::Point point_ld(width * (start_y_ratio + width_ratio), height * (start_x_ratio + 0.0));
  cv::Point point_rd(width * (start_y_ratio + width_ratio), height * (start_x_ratio + height_ratio));
  indicate_polygon_ = { point_lu, point_ru, point_rd, point_ld };

  cv::line(img, point_lu, point_ru, cv::Scalar(0, 0xff, 0), 8);
  cv::line(img, point_ru, point_rd, cv::Scalar(0, 0xff, 0), 8);
  cv::line(img, point_rd, point_ld, cv::Scalar(0, 0xff, 0), 8);
  cv::line(img, point_ld, point_lu, cv::Scalar(0, 0xff, 0), 8);
}
