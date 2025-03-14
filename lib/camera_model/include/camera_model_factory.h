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

#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>  // cv::Size

namespace robosense {
/**
 * @brief Enum of camera type.
 * @var: kNormalPinhole normal pinhole camera mode, with 5 distortion
 * coefficients (k1, k2, p1, p2, k3)
 * @var: kWidePinhole pinhole of wide FOV camera mode, with 8 distortion
 * coefficients (k1, k2, p1, p2, k3, k4, k5, k6)
 * @var: kFisheye: fisheye camera mode, with 4 distortion coefficients (k1, k2,
 * k3, k4)
 */
enum class CameraModelType : int { kNormalPinhole = 0, kWidePinhole, kFisheye };

class CameraModel;
class CameraModelFactory {
 public:
  std::unique_ptr<CameraModel> GetCameraModel(
      CameraModelType camera_model_type);
  std::unique_ptr<CameraModel> GetCameraModel(
      const cv::Mat& camera_matrix, const cv::Mat& distort_coe,
      const cv::Size& image_size, CameraModelType camera_model_type);
};
}  // namespace robosense
