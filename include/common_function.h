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

#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H

#include <eigen3/Eigen/Dense>
#include <vector>
// #include "common_variable.h"
#include <algorithm>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
// #include <dirent.h>

namespace robosense {
namespace calib {
namespace factory_calibration {

struct Pose {
  double t_x = 0.0;
  double t_y = 0.0;
  double t_z = 0.0;
  double q_x = 0.0;
  double q_y = 0.0;
  double q_z = 0.0;
  double q_w = 0.0;
  Eigen::Vector3d tran_vec;
  Eigen::Vector3d rot_vec;  // rot-axis
  Eigen::Vector3d euler_vec;
  Eigen::Quaterniond quaternion;
  Eigen::Matrix3d rotation_matrix;
};
/*
 * revert the nesting vector
 */
template <typename T>
std::vector<std::vector<T>> revertVector(std::vector<std::vector<T>> matrix) {
  std::vector<std::vector<T>> matrix_rev(matrix[0].size(), std::vector<T>());
  for (int i = 0; i < matrix.size(); i++) {
    if (matrix[i].size() == matrix[0].size()) {
      for (int j = 0; j < matrix[0].size(); j++) {
        matrix_rev[j].push_back(matrix[i][j]);
      }
    } else {
      int temp = matrix[0].size() - matrix[i].size();
      for (int j = 0; j < matrix[0].size() - temp; j++) {
        matrix_rev[j].push_back(matrix[i][j]);
      }
    }
  }
  return matrix_rev;
}

/*
 * Transition between 4*4 Transform-Matrix and 6-dof(r-t) Pose-Vector(Euler)
 */

Pose getPoseFromMatrix(Eigen::Matrix4d trans_matrix);
std::vector<double> getEulerPoseFromMatrix(Eigen::Matrix4d trans_matrix);
Eigen::Matrix4d getMatrixFromEulerPose(std::vector<double> pose);
Eigen::Matrix4d getMatrixFromRotationPose(std::vector<double> pose);

int getFileInFolder(const char* dir_name, std::string extend_name, std::vector<std::string>& file_names);
void cubicSplineInterpolation(const std::vector<Eigen::Vector2d>& data,
                              const std::vector<double>& cond,
                              std::vector<double>& coeffs);
std::vector<double> getColour(double v, double vmin, double vmax);

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense
#endif