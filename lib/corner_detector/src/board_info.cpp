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

#include "board_info.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace robosense {
namespace calib {
void operator>>(const YAML::Node& _node, ChAprilBoardInfo& _info) {
  _info.board_type = _node["ch_april_board_type"].as<int>();
  _info.family     = _node["family"].as<std::string>();

  std::vector<int> vec_int(_node["ch_april_layout"].as<std::vector<int>>());
  _info.layout = cv::Size(vec_int[1], vec_int[0]);

  _info.chess_len   = _node["chess_len"].as<double>();
  _info.tag_size    = _node["tag_size"].as<double>();
  _info.tag_spacing = _node["tag_spacing"].as<double>();
  if (_node["tag_area_threshold"]) {
    _info.tag_area_threshold = _node["tag_area_threshold"].as<double>();
  }

  std::vector<double> vec_double(_node["board_size"].as<std::vector<double>>());
  _info.board_size = cv::Size2d(vec_double[1], vec_double[0]);

  std::vector<std::vector<double>> vv;
  vv = _node["board_vertex_offset"].as<std::vector<std::vector<double>>>();
  for (std::size_t i = 0; i < vv.size() && i < _info.board_vertex_offset.size(); ++i) {
    if (2 == vv[i].size()) {
      _info.board_vertex_offset[i] = cv::Point2d(vv[i][1], vv[i][0]);
    } else {
      std::cout << "board_vertex_offset is not like (y_offset, x_offset), please check yaml config file.";
    }
  }

  _info.tag_range         = _node["april_tag_range"].as<std::array<int, 2>>();
  _info.min_valid_tag_num = _node["min_valid_tag_num"].as<int>();
}

void ChAprilBoardInfo::print() const {
  std::string str = "ChAprilBoardInfo: ";
  str += std::string("\nch_april_board_type: ") + std::to_string(board_type);
  str += std::string("\nfamily: ") + family;
  str +=
    std::string("\nch_april_layout: [") + std::to_string(layout.height) + ", " + std::to_string(layout.width) + "]";
  str += std::string("\nchess_len: ") + std::to_string(chess_len);
  str += std::string("\ntag_size: ") + std::to_string(tag_size);
  str += std::string("\ntag_spacing: ") + std::to_string(tag_spacing);
  str += std::string("\ntag_area_threshold: ") + std::to_string(tag_area_threshold);
  str +=
    std::string("\nboard_size: [") + std::to_string(board_size.height) + ", " + std::to_string(board_size.width) + "]";
  str += std::string("\nboard_vertex_offset: [");
  for (std::size_t i = 0; i < board_vertex_offset.size(); ++i) {
    if (i < board_vertex_offset.size() - 1) {
      str += "[" + std::to_string(board_vertex_offset[i].height) + ", " + std::to_string(board_vertex_offset[i].width) +
             "], ";
    } else {
      str += "[" + std::to_string(board_vertex_offset[i].height) + ", " + std::to_string(board_vertex_offset[i].width) +
             "]]";
    }
  }
  str += std::string("\ntag_range: [") + std::to_string(tag_range[0]) + ", " + std::to_string(tag_range[1]) + "]";
  str += std::string("\nmin_valid_tag_num: ") + std::to_string(min_valid_tag_num);
  str += std::string("\n");
  std::cout << (str);
}

bool readChAprilConfig(const std::string& _config_yaml_file, std::vector<ChAprilBoardInfo>& _ch_april_info) {
  bool is_ok = false;
  _ch_april_info.clear();

  if (_config_yaml_file.empty()) {
    std::cout << ("readChAprilConfig: yaml file name is empty");
    return is_ok;
  }

  YAML::Node config = YAML::LoadFile(_config_yaml_file);
  for (std::size_t i = 0; i < config.size(); ++i) {
    ChAprilBoardInfo info;
    config[i] >> info;
    info.print();
    _ch_april_info.emplace_back(info);
  }
  is_ok = _ch_april_info.size() > 0;
  return is_ok;
}
}  // namespace calib
}  // namespace robosense
