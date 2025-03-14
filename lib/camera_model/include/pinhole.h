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
