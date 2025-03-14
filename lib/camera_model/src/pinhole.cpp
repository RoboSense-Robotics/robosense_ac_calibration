/******************************************************************************
 * Copyright 2022 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "pinhole.h"

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace robosense {
PinholeModel::PinholeModel(const cv::Mat& camera_matrix, const cv::Mat& distort_coe, const cv::Size& image_size) :
  CameraModel(camera_matrix, distort_coe, image_size) {
}

double PinholeModel::CalibrateCamera(const std::vector<std::vector<cv::Point3d>>& object_point_3d,
                                     const std::vector<std::vector<cv::Point2d>>& image_point_2d,
                                     const cv::Size& image_size,
                                     std::vector<std::vector<double>>& rvecs,
                                     std::vector<std::vector<double>>& tvecs,
                                     std::vector<double>& projection_error) {
  // init image_size
  image_size_ = image_size;
  std::vector<std::vector<cv::Point3f>> calib_point_3d;
  std::vector<std::vector<cv::Point2f>> calib_point_2d;
  for (size_t i = 0; i < object_point_3d.size(); i++) {
    std::vector<cv::Point3f> point_3d;
    std::vector<cv::Point2f> point_2d;
    for (size_t j = 0; j < object_point_3d[i].size(); j++) {
      point_3d.push_back(static_cast<cv::Point3f>(object_point_3d[i][j]));
      point_2d.push_back(static_cast<cv::Point2f>(image_point_2d[i][j]));
    }
    calib_point_3d.push_back(point_3d);
    calib_point_2d.push_back(point_2d);
  }

  std::size_t min_number_of_views = 15;
  if (object_point_3d.size() < min_number_of_views) {
    std::cerr << "There are too less pattern views for CalibrateCamera: " << object_point_3d.size()
              << ", which must be larger than " << min_number_of_views << std::endl;
    return -1;
  }
  cv::Mat std1, std2, std3;
  std::vector<cv::Mat> r_vecs, t_vecs;
  double error = cv::calibrateCamera(calib_point_3d, calib_point_2d, image_size, camera_matrix_, distort_coefficient_,
                                     r_vecs, t_vecs, std1, std2, std3, calib_flag_);

  for (size_t i = 0; i < r_vecs.size(); i++) {
    rvecs.push_back({ r_vecs[i].at<double>(0), r_vecs[i].at<double>(1), r_vecs[i].at<double>(2) });
    tvecs.push_back({ t_vecs[i].at<double>(0), t_vecs[i].at<double>(1), t_vecs[i].at<double>(2) });
  }

  projection_error.resize(object_point_3d.size(), -1);
  for (std::size_t i = 0; i < object_point_3d.size(); ++i) {
    std::vector<cv::Point2d> project_points;
    ProjectPoints(object_point_3d[i], rvecs[i], tvecs[i], project_points);

    double error_per_board = 0;
    for (std::size_t j = 0; j < project_points.size(); ++j) {
      error_per_board +=
        std::sqrt((project_points[j].x - image_point_2d[i][j].x) * (project_points[j].x - image_point_2d[i][j].x) +
                  (project_points[j].y - image_point_2d[i][j].y) * (project_points[j].y - image_point_2d[i][j].y));
    }

    projection_error[i] = error_per_board / project_points.size();
  }
  return error;
}

void PinholeModel::UndistortImage(const cv::Mat& src, double alpha, cv::Mat& dst) {
  // init map_x, map_y
  if (map_x_.data == nullptr || map_y_.data == nullptr) {
    cv::Mat new_camera_martrix =
      cv::getOptimalNewCameraMatrix(camera_matrix_, distort_coefficient_, image_size_, alpha, image_size_);
    // using CV_16SC2 is 2x faster than CV_32FCx
    cv::initUndistortRectifyMap(camera_matrix_, distort_coefficient_, cv::Mat(), new_camera_martrix, image_size_,
                                CV_16SC2, map_x_, map_y_);
  }
  cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR);
}

void PinholeModel::UndistortPoints(const std::vector<cv::Point2d>& src, std::vector<cv::Point2d>& dst) {
  cv::undistortPoints(src, dst, camera_matrix_, distort_coefficient_);
}

void PinholeModel::ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                                 const cv::Mat& rvec,
                                 const cv::Mat& tvec,
                                 std::vector<cv::Point2d>& point_2d) {
  cv::projectPoints(point_3d, rvec, tvec, camera_matrix_, distort_coefficient_, point_2d);
}

void PinholeModel::ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                                 const std::vector<double>& rvec,
                                 const std::vector<double>& tvec,
                                 std::vector<cv::Point2d>& point_2d) {
  cv::projectPoints(point_3d, rvec, tvec, camera_matrix_, distort_coefficient_, point_2d);
}

void PinholeModel::SolvePnP(const std::vector<cv::Point3d>& point_3d,
                            const std::vector<cv::Point2d>& point_2d,
                            std::vector<double>& rvec,
                            std::vector<double>& tvec) {
  rvec.clear();
  tvec.clear();
  cv::solvePnP(point_3d, point_2d, camera_matrix_, distort_coefficient_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

void PinholeModel::SolvePnPGeneric(const std::vector<cv::Point3d>& point_3d,
                                   const std::vector<cv::Point2d>& point_2d,
                                   std::vector<double>& rvec,
                                   std::vector<double>& tvec) {
  rvec.clear();
  tvec.clear();

  std::vector<cv::Mat> rvecs, tvecs;
  int solutions = cv::solvePnPGeneric(point_3d, point_2d, camera_matrix_, distort_coefficient_, rvecs, tvecs, false,
                                      cv::SOLVEPNP_IPPE);
  std::cout << "camera_matrix:\n" << camera_matrix_ << std::endl;
  std::cout << "distort_coefficient_:\n" << distort_coefficient_ << std::endl;
  std::cout << "point_3d:" << std::endl;
  for (auto p : point_3d) {
    std::cout << p << std::endl;
  }
  std::cout << "point_2d:" << std::endl;
  for (auto p : point_2d) {
    std::cout << p << std::endl;
  }

  for (int i = 0; i < solutions; i++) {
    // 提取平移向量
    cv::Mat tvec = tvecs[i];
    double z     = tvec.at<double>(2);  // 假设 tvec 是 3x1 的 Mat
    std::cout << "solution " << i << std::endl;
    std::cout << tvec << std::endl;

    // 筛选 Z < 0 的解
    if (z < 0) {
      // 使用当前解
      cv::Mat selected_rvec = rvecs[i];
      cv::Mat selected_tvec = tvec;
      break;
    }
  }
}

}  // namespace robosense
