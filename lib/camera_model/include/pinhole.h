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

#pragma once
#include <vector>

#include "camera_model.h"

namespace robosense {

class PinholeModel : public CameraModel {
public:
  PinholeModel() = default;
  PinholeModel(const cv::Mat& camera_matrix, const cv::Mat& distort_coe, const cv::Size& image_size);
  virtual ~PinholeModel() = default;

  /**
   * @brief calibrate camera using 3D-2D point pairs. Camera matrix and
   * distortion coefficients can be accessed by public API.
   *
   * @param object_point_3d 3D points of calibration pattern in the calibration
   * pattern coordinate space
   * @param image_point_2d image points
   * @param image_size image size
   * @param projection_error the mean projection error of each board
   * @return double the overall RMS re-projection error
   */
  double CalibrateCamera(const std::vector<std::vector<cv::Point3d>>& object_point_3d,
                         const std::vector<std::vector<cv::Point2d>>& image_point_3d,
                         const cv::Size& image_size,
                         std::vector<std::vector<double>>& rvecs,
                         std::vector<std::vector<double>>& tvecs,
                         std::vector<double>& projection_error) override;

  void UndistortImage(const cv::Mat& src, double alpha, cv::Mat& dst) override;
  void UndistortPoints(const std::vector<cv::Point2d>& src, std::vector<cv::Point2d>& dst) override;

  void ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                     const cv::Mat& rvec,
                     const cv::Mat& tvec,
                     std::vector<cv::Point2d>& point_2d) override;
  void ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                     const std::vector<double>& rvec,
                     const std::vector<double>& tvec,
                     std::vector<cv::Point2d>& point_2d) override;

  void SolvePnP(const std::vector<cv::Point3d>& point_3d,
                const std::vector<cv::Point2d>& point_2d,
                std::vector<double>& rvec,
                std::vector<double>& tvec) override;
  void SolvePnPGeneric(const std::vector<cv::Point3d>& point_3d,
                       const std::vector<cv::Point2d>& point_2d,
                       std::vector<double>& rvec,
                       std::vector<double>& tvec) override;

protected:
  cv::Mat map_x_;
  cv::Mat map_y_;
};
}  // namespace robosense
