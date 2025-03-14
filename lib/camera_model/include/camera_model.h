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

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>  // cv::Size

namespace robosense {
class CameraModel {
 public:
  virtual ~CameraModel() = default;
  /**
   * @brief Calibrate camera using 3D-2D point pairs. Camera matrix and
   * distortion coefficients can be accessed by public API.
   *
   * @param object_point_3d 3D points of calibration pattern in the calibration
   * pattern coordinate space
   * @param image_point_2d image points
   * @param image_size image size
   * @param projection_error the mean projection error of each board
   * @return double the overall RMS re-projection error
   */
  virtual double CalibrateCamera(
      const std::vector<std::vector<cv::Point3d>>& object_point_3d,
      const std::vector<std::vector<cv::Point2d>>& image_point_3d,
      const cv::Size& image_size, std::vector<std::vector<double>>& rvecs,
      std::vector<std::vector<double>>& tvecs,
      std::vector<double>& projection_error) = 0;

  /**
   * @brief Undistort image based on camera intrinsic parameters.
   *
   * @param src input (distorted) image
   * @param alpha free scaling parameter between 0 (when all the pixels in the
   * undistorted image are valid) and 1 (when all the source image pixels are
   * retained in the undistorted image).
   * @param dst output (corrected) image that has the same size and type as src.
   */
  virtual void UndistortImage(const cv::Mat& src, double alpha,
                              cv::Mat& dst) = 0;
  /**
   * @brief Undistort points.
   *
   * @param src points from raw image, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel
   * (CV_32FC2 or CV_64FC2) (or vector<Point2f> ).
   * @param dst Output ideal point coordinates (1xN/Nx1 2-channel or
   * vector<Point2f> ) after undistortion
   */
  virtual void UndistortPoints(const std::vector<cv::Point2d>& src,
                               std::vector<cv::Point2d>& dst) = 0;
  /**
   * @brief Projects 3D points to an image plane.
   *
   * @param point_3d 3D points
   * @param rvec rotation vector from Rodrigues
   * @param tvec translation vector
   * @param point_2d output image points
   */
  virtual void ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                             const cv::Mat& rvec, const cv::Mat& tvec,
                             std::vector<cv::Point2d>& point_2d) = 0;
  /**
   * @brief Projects 3D points to an image plane.
   *
   * @param point_3d 3D points
   * @param rvec rotation vector from Rodrigues
   * @param tvec translation vector
   * @param point_2d output image points
   */
  virtual void ProjectPoints(const std::vector<cv::Point3d>& point_3d,
                             const std::vector<double>& rvec,
                             const std::vector<double>& tvec,
                             std::vector<cv::Point2d>& point_2d) = 0;

  /**
   * @brief Finds an object pose from 3D-2D point correspondences.
   *
   * @param point_3d 3D points
   * @param rvec rotation vector from Rodrigues
   * @param tvec translation vector
   * @param point_2d point_3d corresponding image points
   */
  virtual void SolvePnP(const std::vector<cv::Point3d>& point_3d,
                        const std::vector<cv::Point2d>& point_2d,
                        std::vector<double>& rvec,
                        std::vector<double>& tvec) = 0;
  virtual void SolvePnPGeneric(const std::vector<cv::Point3d>& point_3d,
                               const std::vector<cv::Point2d>& point_2d,
                               std::vector<double>& rvec,
                               std::vector<double>& tvec) = 0;

  /**
   * @brief Set the camera calibration flag for calibrateCamera.
   *
   * @param flag calibration flag such as cv::CALIB_RATIONAL_MODEL
   */
  void set_calib_flag(int flag) { calib_flag_ = flag; }

  int calib_flag() const { return calib_flag_; }
  cv::Size image_size() const { return image_size_; }
  cv::Mat camera_matrix() const { return camera_matrix_; }
  cv::Mat distort_coefficient() const { return distort_coefficient_; }

 protected:
  CameraModel() = default;
  CameraModel(const cv::Mat& camera_matrix, const cv::Mat& distort_coe,
              const cv::Size& image_size)
      : camera_matrix_(camera_matrix),
        distort_coefficient_(distort_coe),
        image_size_(image_size) {}

  int calib_flag_ = 0;
  cv::Mat camera_matrix_;
  cv::Mat distort_coefficient_;
  cv::Size image_size_;
};
}  // namespace robosense
