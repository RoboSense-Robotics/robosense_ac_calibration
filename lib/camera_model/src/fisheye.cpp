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

#include "fisheye.h"

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace robosense {
FisheyeModel::FisheyeModel(const cv::Mat& camera_matrix,
                           const cv::Mat& distort_coe,
                           const cv::Size& image_size)
    : CameraModel(camera_matrix, distort_coe, image_size) {}

double FisheyeModel::CalibrateCamera(
    const std::vector<std::vector<cv::Point3d>>& object_point_3d,
    const std::vector<std::vector<cv::Point2d>>& image_point_2d,
    const cv::Size& image_size, std::vector<std::vector<double>>& rvecs,
    std::vector<std::vector<double>>& tvecs,
    std::vector<double>& projection_error) {
  // init image_size
  image_size_ = image_size;
  size_t min_count = 1000;
  for (auto point_vec : object_point_3d) {
    min_count = std::min(point_vec.size(), min_count);
  }

  std::vector<std::vector<cv::Point3f>> calib_point_3d;
  std::vector<std::vector<cv::Point2f>> calib_point_2d;
  for (size_t i = 0; i < object_point_3d.size(); i++) {
    std::vector<cv::Point3f> point_3d;
    std::vector<cv::Point2f> point_2d;
    for (size_t j = 0; j < min_count; j++) {
      point_3d.push_back(static_cast<cv::Point3f>(object_point_3d[i][j]));
      point_2d.push_back(static_cast<cv::Point2f>(image_point_2d[i][j]));
    }
    calib_point_3d.push_back(point_3d);
    calib_point_2d.push_back(point_2d);
  }

  std::size_t min_number_of_views = 15;
  if (object_point_3d.size() < min_number_of_views) {
    std::cerr << "There are too less pattern views for CalibrateCamera: "
              << object_point_3d.size() << ", which must be larger than "
              << min_number_of_views << std::endl;
    return -1;
  }

  std::vector<cv::Mat> r_vecs, t_vecs;
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  // camera_matrix_.at(0, 0);
  // camera_matrix_.at(1, 1);
  camera_matrix_.at<double>(0, 2) = image_size.width / 2.0;
  camera_matrix_.at<double>(1, 2) = image_size.height / 2.0;
  double error = cv::fisheye::calibrate(
      calib_point_3d, calib_point_2d, image_size, camera_matrix_,
      distort_coefficient_, r_vecs, t_vecs, calib_flag_);

  for (size_t i = 0; i < r_vecs.size(); i++) {
    rvecs.push_back({r_vecs[i].at<double>(0), r_vecs[i].at<double>(1),
                     r_vecs[i].at<double>(2)});
    tvecs.push_back({t_vecs[i].at<double>(0), t_vecs[i].at<double>(1),
                     t_vecs[i].at<double>(2)});
  }

  projection_error.resize(object_point_3d.size(), -1);
  for (std::size_t i = 0; i < object_point_3d.size(); ++i) {
    std::vector<cv::Point2d> project_points;
    ProjectPoints(object_point_3d[i], rvecs[i], tvecs[i], project_points);

    double error_per_board = 0;
    for (std::size_t j = 0; j < project_points.size(); ++j) {
      error_per_board +=
          std::sqrt((project_points[j].x - image_point_2d[i][j].x) *
                        (project_points[j].x - image_point_2d[i][j].x) +
                    (project_points[j].y - image_point_2d[i][j].y) *
                        (project_points[j].y - image_point_2d[i][j].y));
    }

    projection_error[i] = error_per_board / project_points.size();
  }
  return error;
}

void FisheyeModel::UndistortImage(const cv::Mat& src, double alpha,
                                  cv::Mat& dst) {
  // init map_x, map_y
  if (map_x_.data == nullptr || map_y_.data == nullptr) {
    // using CV_16SC2 is 2x faster than CV_32FCx
    cv::fisheye::initUndistortRectifyMap(camera_matrix_, distort_coefficient_,
                                         cv::Mat(), camera_matrix_, image_size_,
                                         CV_16SC2, map_x_, map_y_);
  }
  cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR);
}

void FisheyeModel::UndistortPoints(const std::vector<cv::Point2d>& src,
                                   std::vector<cv::Point2d>& dst) {
  cv::fisheye::undistortPoints(src, dst, camera_matrix_, distort_coefficient_);
}

void FisheyeModel::ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                                 const cv::Mat& rvec, const cv::Mat& tvec,
                                 std::vector<cv::Point2d>& point_2d) {
  cv::fisheye::projectPoints(point_3d, point_2d, rvec, tvec, camera_matrix_,
                             distort_coefficient_);
}
void FisheyeModel::ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                                 const std::vector<double>& rvec,
                                 const std::vector<double>& tvec,
                                 std::vector<cv::Point2d>& point_2d) {
  cv::fisheye::projectPoints(point_3d, point_2d, rvec, tvec, camera_matrix_,
                             distort_coefficient_);
}

void FisheyeModel::SolvePnP(const std::vector<cv::Point3d>& point_3d,
                            const std::vector<cv::Point2d>& point_2d,
                            std::vector<double>& rvec,
                            std::vector<double>& tvec) {
  std::vector<cv::Point2d> undistort_point_2d;
  rvec.clear();
  tvec.clear();
  UndistortPoints(point_2d, undistort_point_2d);
  cv::solvePnP(point_3d, undistort_point_2d, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 4, CV_64F), rvec, tvec,
               false, cv::SOLVEPNP_EPNP);
}

void FisheyeModel::SolvePnPGeneric(const std::vector<cv::Point3d>& point_3d,
                                   const std::vector<cv::Point2d>& point_2d,
                                   std::vector<double>& rvec,
                                   std::vector<double>& tvec) {
  // TODO: 
  std::vector<cv::Point2d> undistort_point_2d;
  rvec.clear();
  tvec.clear();
  UndistortPoints(point_2d, undistort_point_2d);
  cv::solvePnP(point_3d, undistort_point_2d, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 4, CV_64F), rvec, tvec,
               false, cv::SOLVEPNP_EPNP);
}

}  // namespace robosense
