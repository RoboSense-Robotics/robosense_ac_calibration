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

#ifndef CORNER_POINT_DETECTOR_BOARD_INFO_H
#define CORNER_POINT_DETECTOR_BOARD_INFO_H

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

namespace robosense
{
namespace calib
{
struct ChAprilBoardInfo
{
  int board_type;  // valid number from 1
  int min_valid_tag_num;
  float chess_len;
  std::array<int, 2> tag_range;
  double tag_size;
  double tag_spacing;
  double tag_area_threshold;
  cv::Size layout;
  cv::Size2d board_size;
  std::array<cv::Size2d, 4> board_vertex_offset;
  std::string family;
  ChAprilBoardInfo() :
    board_type(0), min_valid_tag_num(std::numeric_limits<int>::max()), chess_len(-1), tag_area_threshold(0), family("")
  {}
  void print() const;
};

bool readChAprilConfig(const std::string& _config_yaml_file, std::vector<ChAprilBoardInfo>& _ch_april_info);
// class ChAprilConfig
// {
// public:
//   ChAprilConfig();
//   ~ChAprilConfig() = default;
//   bool read(const std::string& _config_yaml_file, std::vector<ChAprilBoardInfo>& _ch_april_info);
// };  // class ChAprilConfig
}  // namespace calib
}  // namespace robosense
#endif  // CORNER_POINT_DETECTOR_BOARD_INFO_H
