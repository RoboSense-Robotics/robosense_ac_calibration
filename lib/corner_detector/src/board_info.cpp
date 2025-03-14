﻿/******************************************************************************
 * Copyright 2023 RoboSense All rights reserved.
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
