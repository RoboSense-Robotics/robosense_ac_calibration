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

#include "camera_model_factory.h"

#include <iostream>

#include <opencv2/calib3d.hpp>

#include "camera_model.h"
#include "fisheye.h"
#include "pinhole.h"

namespace robosense {
std::unique_ptr<CameraModel> CameraModelFactory::GetCameraModel(
    CameraModelType camera_model_type) {
  std::unique_ptr<CameraModel> camera_model(nullptr);
  switch (camera_model_type) {
    case CameraModelType::kNormalPinhole: {
      camera_model = std::make_unique<PinholeModel>();
      break;
    }
    case CameraModelType::kWidePinhole: {
      camera_model = std::make_unique<PinholeModel>();
      camera_model->set_calib_flag(cv::CALIB_RATIONAL_MODEL);
      break;
    }
    case CameraModelType::kFisheye: {
      camera_model = std::make_unique<FisheyeModel>();
      camera_model->set_calib_flag(cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC +
                                   cv::fisheye::CALIB_CHECK_COND +
                                   cv::fisheye::CALIB_FIX_SKEW);
      break;
    }
    default: {
      std::cout << "undefined camera model, please check" << std::endl;
      break;
    }
  }
  return camera_model;
}

std::unique_ptr<CameraModel> CameraModelFactory::GetCameraModel(
    const cv::Mat& camera_matrix, const cv::Mat& distort_coe,
    const cv::Size& image_size, CameraModelType camera_model_type) {
  std::unique_ptr<CameraModel> camera_model(nullptr);
  switch (camera_model_type) {
    case CameraModelType::kNormalPinhole: {
      camera_model = std::make_unique<PinholeModel>(camera_matrix, distort_coe,
                                                    image_size);
      break;
    }
    case CameraModelType::kWidePinhole: {
      camera_model = std::make_unique<PinholeModel>(camera_matrix, distort_coe,
                                                    image_size);
      camera_model->set_calib_flag(cv::CALIB_RATIONAL_MODEL);
      break;
    }
    case CameraModelType::kFisheye: {
      camera_model = std::make_unique<FisheyeModel>(camera_matrix, distort_coe,
                                                    image_size);
      camera_model->set_calib_flag(cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC +
                                   cv::fisheye::CALIB_CHECK_COND +
                                   cv::fisheye::CALIB_FIX_SKEW);
      break;
    }
    default: {
      std::cout << "undefined camera model, please check" << std::endl;
      break;
    }
  }
  return camera_model;
}

}  // namespace robosense
