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
  std::unique_ptr<CameraModel> GetCameraModel(CameraModelType camera_model_type);
  std::unique_ptr<CameraModel> GetCameraModel(const cv::Mat& camera_matrix,
                                              const cv::Mat& distort_coe,
                                              const cv::Size& image_size,
                                              CameraModelType camera_model_type);
};
}  // namespace robosense
