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

#include "common_function.h"
namespace robosense {
namespace calib {
namespace factory_calibration {

bool CompareFilenames(const std::string& filename1, const std::string& filename2) {
  if (filename1.length() < filename2.length()) {
    return true;
  } else if (filename1.length() > filename2.length()) {
    return false;
  } else {
    return filename1 < filename2;
  }
}

int getFileInFolder(const char* dir_name, std::string extend_name, std::vector<std::string>& file_names) {
  int number = 0;
  // check the parameter !
  if (NULL == dir_name) {
    std::cout << " dir_name is null ! " << std::endl;
    return 0;
  }

  DIR* dir = opendir(dir_name);
  if (NULL == dir) {
    std::cout << "Can not open dir " << dir_name << std::endl;
    return 0;
  }

  /* read all the files in the dir ~ */
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
      continue;
    }
    file_names.push_back(std::string(entry->d_name));
    number++;
  }
  closedir(dir);
  std::sort(file_names.begin(), file_names.end(), CompareFilenames);
  return number;
}

Pose getPoseFromMatrix(Eigen::Matrix4d trans_matrix) {
  Pose pose;
  Eigen::Vector3d tran_vec        = trans_matrix.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation_matrix = trans_matrix.block(0, 0, 3, 3);

  pose.tran_vec        = tran_vec;
  pose.rotation_matrix = rotation_matrix;

  pose.t_x = tran_vec(0);
  pose.t_y = tran_vec(1);
  pose.t_z = tran_vec(2);

  pose.euler_vec = pose.rotation_matrix.eulerAngles(2, 1, 0);
  // TODO axis-angle
  pose.quaternion = Eigen::Quaterniond(rotation_matrix).normalized();

  pose.q_w = pose.quaternion.w();
  pose.q_x = pose.quaternion.x();
  pose.q_y = pose.quaternion.y();
  pose.q_z = pose.quaternion.z();
  return pose;
}

std::vector<double> getEulerPoseFromMatrix(Eigen::Matrix4d trans_matrix) {
  std::vector<double> pose;
  Eigen::Matrix3d rotation_matrix = trans_matrix.block(0, 0, 3, 3);
  // roll pitch yaw order(Rz*Ry*Rx)
  Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(2, 1, 0);

  pose.push_back(euler_angle(2));
  pose.push_back(euler_angle(1));
  pose.push_back(euler_angle(0));
  pose.push_back(trans_matrix(0, 3));
  pose.push_back(trans_matrix(1, 3));
  pose.push_back(trans_matrix(2, 3));
  return pose;
}

Eigen::Matrix4d getMatrixFromEulerPose(std::vector<double> pose) {
  Eigen::Matrix4d trans_matrix;
  // yaw pitch roll order
  Eigen::Vector3d euler_angle;
  euler_angle << pose[0], pose[1], pose[2];
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;

  trans_matrix.setIdentity();
  trans_matrix.block(0, 0, 3, 3) = rotation_matrix;

  trans_matrix(0, 3) = pose[3];
  trans_matrix(1, 3) = pose[4];
  trans_matrix(2, 3) = pose[5];
  return trans_matrix;
}

Eigen::Matrix4d getMatrixFromRotationPose(std::vector<double> pose) {
  Eigen::Matrix4d trans_matrix;
  // yaw pitch roll order
  Eigen::Vector3d rotation_vec;
  rotation_vec << pose[0], pose[1], pose[2];

  double angle         = rotation_vec.norm();        // 旋转角度
  Eigen::Vector3d axis = rotation_vec.normalized();  // 单位化的旋转轴

  // 使用 AngleAxis 表示旋转
  Eigen::AngleAxisd angle_axis(angle, axis);

  // 转换为旋转矩阵
  Eigen::Matrix3d rotation_matrix = angle_axis.toRotationMatrix();

  trans_matrix.setIdentity();
  trans_matrix.block(0, 0, 3, 3) = rotation_matrix;

  trans_matrix(0, 3) = pose[3];
  trans_matrix(1, 3) = pose[4];
  trans_matrix(2, 3) = pose[5];
  return trans_matrix;
}

void cubicSplineInterpolation(const std::vector<Eigen::Vector2d>& data,
                              const std::vector<double>& cond,
                              std::vector<double>& coeffs) {
  size_t num_coeff = data.size() - 1;
  size_t nrows = num_coeff * 4, ncols = num_coeff * 4;
  coeffs.resize(num_coeff * 4);
  Eigen::MatrixXd x_values = Eigen::MatrixXd::Zero(nrows, ncols);
  Eigen::MatrixXd y_values = Eigen::MatrixXd::Zero(nrows, 1);

  size_t i = 0, j = 1;
  for (; i < data.size() - 2 and j < data.size() - 1; ++i, ++j) {
    x_values(i * 4, i * 4)     = std::pow(data[i].x(), 3);
    x_values(i * 4, i * 4 + 1) = std::pow(data[i].x(), 2);
    x_values(i * 4, i * 4 + 2) = data[i].x();
    x_values(i * 4, i * 4 + 3) = 1.0;
    y_values(i * 4, 0)         = data[i].y();

    x_values(i * 4 + 1, i * 4)     = std::pow(data[j].x(), 3);
    x_values(i * 4 + 1, i * 4 + 1) = std::pow(data[j].x(), 2);
    x_values(i * 4 + 1, i * 4 + 2) = data[j].x();
    x_values(i * 4 + 1, i * 4 + 3) = 1.0;
    y_values(i * 4 + 1, 0)         = data[j].y();

    x_values(i * 4 + 2, i * 4)           = 3 * std::pow(data[j].x(), 2);
    x_values(i * 4 + 2, i * 4 + 1)       = 2 * data[j].x();
    x_values(i * 4 + 2, i * 4 + 2)       = 1.0;
    x_values(i * 4 + 2, (i + 1) * 4)     = -3 * std::pow(data[j].x(), 2);
    x_values(i * 4 + 2, (i + 1) * 4 + 1) = -2 * data[j].x();
    x_values(i * 4 + 2, (i + 1) * 4 + 2) = -1.0;
    y_values(i * 4 + 2, 0)               = 0;

    x_values(i * 4 + 3, i * 4)           = 6 * data[j].x();
    x_values(i * 4 + 3, i * 4 + 1)       = 2.0;
    x_values(i * 4 + 3, (i + 1) * 4)     = -6 * data[j].x();
    x_values(i * 4 + 3, (i + 1) * 4 + 1) = -2.0;
    y_values(i * 4 + 3, 0)               = 0;
  }

  x_values(i * 4, i * 4)     = std::pow(data[i].x(), 3);
  x_values(i * 4, i * 4 + 1) = std::pow(data[i].x(), 2);
  x_values(i * 4, i * 4 + 2) = data[i].x();
  x_values(i * 4, i * 4 + 3) = 1.0;
  y_values(i * 4, 0)         = data[i].y();

  x_values(i * 4 + 1, i * 4)     = std::pow(data[i + 1].x(), 3);
  x_values(i * 4 + 1, i * 4 + 1) = std::pow(data[i + 1].x(), 2);
  x_values(i * 4 + 1, i * 4 + 2) = data[i + 1].x();
  x_values(i * 4 + 1, i * 4 + 3) = 1.0;
  y_values(i * 4 + 1, 0)         = data[i + 1].y();

  x_values(i * 4 + 2, 0) = 3 * std::pow(data.front().x(), 2);
  x_values(i * 4 + 2, 1) = 2 * data.front().x();
  x_values(i * 4 + 2, 2) = 1;
  y_values(i * 4 + 2, 0) = cond.front();

  x_values(i * 4 + 3, ncols - 4) = 3 * std::pow(data[2].x(), 2);
  x_values(i * 4 + 3, ncols - 3) = 2 * data[2].x();
  x_values(i * 4 + 3, ncols - 2) = 1;
  y_values(i * 4 + 3, 0)         = cond.back();

  Eigen::VectorXd coeff;
  coeff = x_values.colPivHouseholderQr().solve(y_values);

  coeffs.assign(coeff.data(), coeff.data() + ncols);
}

std::vector<double> getColour(double v, double vmin, double vmax) {
  std::vector<double> c = { 1.0, 1.0, 1.0 };  // white
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    c[0] = 0;
    c[1] = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c[0] = 0;
    c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
    c[2] = 0;
  } else {
    c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c[2] = 0;
  }

  return c;
}

}  // namespace factory_calibration
}  // namespace calib
}  // namespace robosense