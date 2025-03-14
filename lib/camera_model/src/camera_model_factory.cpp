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
