/******************************************************************************
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
