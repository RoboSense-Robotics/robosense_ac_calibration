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

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/common/getopt.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
}

#include "corner_point_detector.h"

#include <numeric>
#include <set>

#include <opencv2/opencv.hpp>
namespace robosense {
namespace calib {

void drawApriltags(const std::vector<AprilTagInfo>& _tags, cv::Mat& _img) {
  int font_face     = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double font_scale = 0.5;

  for (const auto& tag : _tags) {
    // red, green, blue, cyan
    cv::line(_img, cv::Point(tag.corners_2d.at(0)), tag.corners_2d.at(1), cv::Scalar(0, 0, 0xff), 2);
    cv::line(_img, cv::Point(tag.corners_2d.at(1)), tag.corners_2d.at(2), cv::Scalar(0, 0xff, 0), 2);
    cv::line(_img, cv::Point(tag.corners_2d.at(2)), tag.corners_2d.at(3), cv::Scalar(0xff, 0, 0), 2);
    cv::line(_img, cv::Point(tag.corners_2d.at(3)), tag.corners_2d.at(0), cv::Scalar(0xff, 0xff, 0), 2);
    std::string text = std::to_string(tag.id);
    cv::putText(_img, text, cv::Point(tag.getTagCenter()), font_face, font_scale, cv::Scalar(0xff, 0x99, 0), 2);
  }
}

void drawApriltags(const std::vector<ChAprilBoardPoints>& _board_points, cv::Mat& _img) {
  for (const auto& board : _board_points) {
    if (board.tags.size() > 0) {
      drawApriltags(board.tags, _img);
    }
  }
}

void drawChessCorners(const std::vector<cv::Point2d>& _point,
                      const std::vector<int>& _corner_index,
                      cv::Mat& _img_dst) {
  int font_face     = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double font_scale = 0.5;
  for (std::size_t i = 0; i < _corner_index.size(); ++i) {
    cv::circle(_img_dst, cv::Point(_point[i]), 5, cv::Scalar(0, 0xff, 0), 2);
    std::string text = std::to_string(_corner_index[i]);
    cv::putText(_img_dst, text, cv::Point(_point[i]), font_face, font_scale, cv::Scalar(0, 0x99, 0), 2);
  }
}

static double getAreaOfTag(const std::array<cv::Point2d, 4>& _corners) {
  // shoelace formula
  double area =
    0.5 * std::abs(_corners[0].x * _corners[1].y + _corners[1].x * _corners[2].y + _corners[2].x * _corners[3].y +
                   _corners[3].x * _corners[0].y - _corners[1].x * _corners[0].y - _corners[2].x * _corners[1].y -
                   _corners[3].x * _corners[2].y - _corners[0].x * _corners[3].y);
  return area;
}

void AprilTagInfo::print() const {
  std::string str = "tag id: " + std::to_string(id) + "\npixel points: ";
  for (const auto& d2 : corners_2d) {
    str += "[" + std::to_string(d2.x) + ", " + std::to_string(d2.y) + "]" + ", ";
  }
  str += "\n 3d points: ";
  for (const auto& d3 : corners_3d) {
    str += "[" + std::to_string(d3.x) + ", " + std::to_string(d3.y) + ", " + std::to_string(d3.z) + "]" + ", ";
  }
  std::cout << (str);
}

DeletedUniquePtr<apriltag_family> ChAprilTagDetector::creatAprilTagFamily(const std::string& _family_name) {
  DeletedUniquePtr<apriltag_family> family = nullptr;
  if ("tagStandard41h12" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tagStandard41h12_create(), tagStandard41h12_destroy);
  } else if ("tag36h11" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tag36h11_create(), tag36h11_destroy);
  } else if ("tag25h9" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tag25h9_create(), tag25h9_destroy);
  } else if ("tag16h5" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tag16h5_create(), tag16h5_destroy);
  } else if ("tagCircle21h7" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tagCircle21h7_create(), tagCircle21h7_destroy);
  } else if ("tagStandard52h13" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tagStandard52h13_create(), tagStandard52h13_destroy);
  } else if ("tagCustom48h12" == _family_name) {
    family = DeletedUniquePtr<apriltag_family_t>(tagCustom48h12_create(), tagCustom48h12_destroy);
  } else {
    std::cout << ("creatAprilTagFamily: undefined tag family: " + _family_name);
  }
  return family;
}

DeletedUniquePtr<apriltag_detector> ChAprilTagDetector::creatAprilTagDetector(apriltag_family* _family) {
  auto tag_detector = DeletedUniquePtr<apriltag_detector_t>(apriltag_detector_create(), apriltag_detector_destroy);
  apriltag_detector_add_family(tag_detector.get(), _family);
  tag_detector->nthreads      = 1;
  tag_detector->refine_edges  = true;
  tag_detector->debug         = false;
  tag_detector->quad_sigma    = 0.2;
  tag_detector->quad_decimate = 0;
  return tag_detector;
}

ChAprilTagDetector::ChAprilTagDetector() : sub_pixel_window_size_(cv::Size(11, 11)) {
}
ChAprilTagDetector::~ChAprilTagDetector() {
}

bool ChAprilTagDetector::init(const std::string& _config_yaml) {
  bool is_ok = readBoardInfo(_config_yaml);
  is_ok &= checkAprilTagValid();
  if (is_ok) {
    families_.clear();
    tag_detectors_.clear();

    std::set<std::string> family_type;
    for (const auto& info : board_infos_) {
      family_type.insert(info.family);
      std::string board_id = makeBoardId(info);
      if (layout_board_tag_3d_corners_.count(board_id) == 0) {
        layout_board_tag_3d_corners_[board_id] = initBoardTagCorners3D(info);
      }
    }
    std::cout << ("    the number of family type is: " + std::to_string(family_type.size()));
    std::cout << ("    the number of layout is: " + std::to_string(layout_board_tag_3d_corners_.size()));
    for (const auto& item : layout_board_tag_3d_corners_) {
      std::cout << ("      layout: " + item.first);
    }

    for (const auto& f : family_type) {
      std::cout << ("    family: " + f);
      families_.emplace_back(creatAprilTagFamily(f));
      if (families_.back().get()) {
        tag_detectors_.emplace_back(creatAprilTagDetector(families_.back().get()));
        if (!tag_detectors_.back()) {
          is_ok = false;
          std::cout << ("ChAprilTagDetector::init fail to create april tag detector. board type: " + f);
        }
      } else {
        is_ok = false;
        std::cout << ("ChAprilTagDetector::init fail to create april tag family: " + f);
      }
    }
  }
  return is_ok;
}

std::map<int, std::array<cv::Point3d, 4>> ChAprilTagDetector::initBoardTagCorners3D(
  const ChAprilBoardInfo& _board_info) {
  int rows           = _board_info.layout.height;
  int cols           = _board_info.layout.width;
  double tag_size    = _board_info.tag_size / 1000;  // mm to meter
  double tag_spacing = _board_info.tag_spacing;
  double chess_len   = _board_info.chess_len / 1000;  // mm t0 meter
  int tag_index      = 0;

  cv::Point3d origin_offset(-chess_len * 0.5, -chess_len * 0.5, 0);
  std::map<int, std::array<cv::Point3d, 4>> corners_3d_map;

  // 3 --- 2
  // |     |
  // 0 --- 1
  std::array<cv::Point3d, 4> tag_corners_3d(
    { cv::Point3d(-tag_size * 0.5, tag_size * 0.5, 0), cv::Point3d(tag_size * 0.5, tag_size * 0.5, 0),
      cv::Point3d(tag_size * 0.5, -tag_size * 0.5, 0), cv::Point3d(-tag_size * 0.5, -tag_size * 0.5, 0) });

  for (int row = 0; row < rows; ++row) {
    for (int col = row % 2; col < cols; col += 2) {
      cv::Point3d center(col * tag_size * (1.0 + tag_spacing), row * tag_size * (1.0 + tag_spacing),
                         0.0);  // origin point at center of first tag
      std::array<cv::Point3d, 4> corners_3d;
      for (std::size_t corner_index = 0; corner_index < tag_corners_3d.size(); ++corner_index) {
        corners_3d[corner_index] = tag_corners_3d[corner_index] + center + origin_offset;
        // std::cout << "[" << corners_3d[corner_index] << "]"
        //           << ", ";
      }
      // std::cout << std::endl;
      corners_3d_map[tag_index] = corners_3d;
      ++tag_index;
    }
    // std::cout << std::endl;
  }
  return corners_3d_map;
}

bool ChAprilTagDetector::readBoardInfo(const std::string& _config_yaml) {
  bool flag = readChAprilConfig(_config_yaml, board_infos_);
  std::string str("ChAprilTagDetector::readBoardInfo: ");
  if (flag) {
    str += "read chess april tag board information successfully: " + _config_yaml;
    std::cout << (str);
  } else {
    str += "fail to read chess april tag board information: " + _config_yaml;
    std::cout << (str);
  }
  return flag;
}

bool ChAprilTagDetector::checkAprilTagValid() {
  bool is_ok = true;
  if (board_infos_.empty()) {
    is_ok = false;
    std::cout << ("ChAprilTagDetector::checkAprilTagValid: board info is empty, please read config file first.");
  } else if (board_infos_.size() > 1) {
    for (std::size_t i = 1; i < board_infos_.size(); ++i) {
      if (board_infos_[i].board_type <= 0 || board_infos_[i].board_type == board_infos_[i - 1].board_type) {
        is_ok = false;
        std::cout << ("ChAprilTagDetector::checkAprilTagValid: board type should be different, and "
                      "larger than 0, such as 1, 2, 3 ... : " +
                      std::to_string(board_infos_[i].board_type) + ", " +
                      std::to_string(board_infos_[i - 1].board_type));
      }
    }
    is_ok &= board_infos_.back().board_type > 0;
  }
  return is_ok;
}

void ChAprilTagDetector::parseTags(const zarray* _detections, cv::Mat& _img) {
  std::vector<ChAprilBoardPoints> one_family_board_point;
  one_family_board_point.resize(board_infos_.size());

  for (int i = 0; i < zarray_size(_detections); i++) {
    apriltag_detection_t* det;
    zarray_get(_detections, i, &det);
    std::array<cv::Point2d, 4> corners = {
      cv::Point2d(det->p[0][0], det->p[0][1]), cv::Point2d(det->p[1][0], det->p[1][1]),
      cv::Point2d(det->p[2][0], det->p[2][1]), cv::Point2d(det->p[3][0], det->p[3][1])
    };  //bottom_left_point, bottom_right_point, top_right_point, top_left_point

    double tag_area = getAreaOfTag(corners);
    for (std::size_t j = 0; j < board_infos_.size(); ++j) {
      if (std::string(det->family->name) == board_infos_[j].family && det->id >= board_infos_[j].tag_range[0] &&
          det->id <= board_infos_[j].tag_range[1] && tag_area > board_infos_[j].tag_area_threshold) {
        AprilTagInfo tag_info;
        tag_info.corners_2d = corners;
        tag_info.id         = det->id;
        tag_info.index      = det->id - board_infos_[j].tag_range[0];
        tag_info.corners_3d = getTagCorners3D(board_infos_[j], det->id);

        one_family_board_point[j].tags.emplace_back(tag_info);
        one_family_board_point[j].board_type = board_infos_[j].board_type;
        one_family_board_point[j].family     = board_infos_[j].family;
      }
    }
  }
  drawApriltags(one_family_board_point, _img);
  for (std::size_t i = 0; i < one_family_board_point.size(); ++i) {
    if (static_cast<int>(one_family_board_point[i].tags.size()) >= board_infos_[i].min_valid_tag_num) {
      board_type_index_[board_infos_[i].board_type] = i;
      board_points_.push_back(one_family_board_point[i]);
    } else {
      std::cout << ("family " + std::to_string(one_family_board_point[i].board_type) +
                    " board too less tags have been detected: " +
                    std::to_string(one_family_board_point[i].tags.size()));
    }
  }
}

bool ChAprilTagDetector::detect(const cv::Mat& _img_src, cv::Mat& _img_dst) {
  bool is_ok = true;
  board_points_.clear();
  board_type_index_.clear();
  if (board_infos_.empty() || families_.empty() || tag_detectors_.empty()) {
    std::cout << ("ChAprilTagDetector::detect: please init first before detect tag image.");
    return false;
  }

  if (!_img_src.data) {
    std::cout << ("ChAprilTagDetector::detect: input image is empty.");
    return false;
  }

  cv::Mat img_gray;
  if (1 == _img_src.channels()) {
    img_gray = _img_src.clone();
    cv::cvtColor(_img_src, _img_dst, cv::COLOR_GRAY2BGR);
  } else if (3 == _img_src.channels()) {
    cv::cvtColor(_img_src, img_gray, cv::COLOR_BGR2GRAY);
    _img_dst = _img_src.clone();
  } else {
    std::cout << ("ChAprilTagDetector::detect: input image channel is invalid: " + std::to_string(_img_src.channels()));
    is_ok = false;
  }

  if (is_ok) {
    image_u8_t img_u8 = { .width  = img_gray.cols,
                          .height = img_gray.rows,
                          .stride = img_gray.cols,
                          .buf =
                            img_gray.data };  // buf refer to cv::Mat data[], cv::Mat will free it when get out of scope
    for (std::size_t i = 0; i < tag_detectors_.size(); ++i) {
      // sorted ascending based on tag id
      auto detections = DeletedUniquePtr<zarray_t>(apriltag_detector_detect(tag_detectors_[i].get(), &img_u8),
                                                   apriltag_detections_destroy);
      if (zarray_size(detections.get()) > 0) {
        parseTags(detections.get(), _img_dst);
      }
    }
    if (board_points_.size() > 0) {
      is_ok &= detectChessCorners(img_gray, _img_dst);
    } else {
      is_ok = false;
      std::cout << ("ChAprilTagDetector::detect: fail to detect tags.");
    }
  }
  return is_ok;
}

cv::Point3d ChAprilTagDetector::getChessCornerPoint3D(const double _chess_len,
                                                      std::size_t _row,
                                                      const std::size_t _col) {
  return cv::Point3d(_col * _chess_len, _row * _chess_len, 0);
}

std::array<cv::Point3d, 4> ChAprilTagDetector::getTagCorners3D(const ChAprilBoardInfo& _board_info, const int _tag_id) {
  std::array<cv::Point3d, 4> corners;
  if (_tag_id >= _board_info.tag_range[0] && _tag_id <= _board_info.tag_range[1]) {
    std::string board_id = makeBoardId(_board_info);
    corners              = layout_board_tag_3d_corners_[board_id][_tag_id - _board_info.tag_range[0]];
  } else {
    std::cout << (std::to_string(_board_info.board_type) +
                  " board doesn't contain this tag: " + std::to_string(_tag_id));
  }
  return corners;
}

std::string ChAprilTagDetector::makeBoardId(const ChAprilBoardInfo& _board_info) {
  return _board_info.family + "_" + std::to_string(_board_info.layout.height) + "_" +
         std::to_string(_board_info.layout.width) + "_" + std::to_string(static_cast<int>(_board_info.tag_size)) + "_" +
         std::to_string(static_cast<int>(_board_info.tag_spacing * 100));
}

int ChAprilTagDetector::getTagIndex(const std::vector<AprilTagInfo>& _vec, const int _tag) {
  int low = 0, high = static_cast<int>(_vec.size()), middle = 0;
  while (low < high) {
    middle = (low + high) / 2;
    if (_tag == _vec[middle].id) {
      return middle;
    } else if (_tag < _vec[middle].id) {
      high = middle;
    } else if (_tag > _vec[middle].id) {
      low = middle + 1;
    }
  }
  return -1;
}

cv::Point2d ChAprilTagDetector::getPattern1CornerPoint(const AprilTagInfo& _top_left,
                                                       const AprilTagInfo& _bottom_right) {
  return (_top_left.corners_2d.at(1) + _bottom_right.corners_2d.at(3)) * 0.5;
}
cv::Point2d ChAprilTagDetector::getPattern2CornerPoint(const AprilTagInfo& _top_right,
                                                       const AprilTagInfo& _bottom_left) {
  return (_top_right.corners_2d.at(0) + _bottom_left.corners_2d.at(2)) * 0.5;
}

void ChAprilTagDetector::convertPoint2fTo2d(const std::vector<cv::Point2f>& _pf, std::vector<cv::Point2d>& _pd) {
  _pd.clear();
  _pd.resize(_pf.size());
  for (std::size_t i = 0; i < _pf.size(); ++i) {
    _pd[i] = cv::Point2d(static_cast<double>(_pf[i].x), static_cast<double>(_pf[i].y));
  }
}

void ChAprilTagDetector::convertPoint2dTo2f(const std::vector<cv::Point2d>& _pd, std::vector<cv::Point2f>& _pf) {
  _pf.clear();
  _pf.resize(_pd.size());
  for (std::size_t i = 0; i < _pd.size(); ++i) {
    _pf[i] = cv::Point2f(static_cast<float>(_pd[i].x), static_cast<float>(_pd[i].y));
  }
}

void ChAprilTagDetector::getCornerSubPix(const cv::Mat& _img_gray, std::vector<cv::Point2d>& _point) {
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                               40,     //maxCount
                                               0.01);  //epsilon
  std::vector<cv::Point2f> pf;
  convertPoint2dTo2f(_point, pf);
  cv::cornerSubPix(_img_gray, pf, sub_pixel_window_size_, cv::Size(-1, -1), criteria);
  convertPoint2fTo2d(pf, _point);
}

bool ChAprilTagDetector::setSubPixelWindowSize(const cv::Size& _size) {
  bool is_ok = false;
  if (_size.width == _size.height && _size.width > 3) {
    sub_pixel_window_size_ = _size;
    is_ok                  = true;
    std::cout << ("Set sub pixel window size successfully: (" + std::to_string(_size.height) + ", " +
                  std::to_string(_size.width) + ")")
              << std::endl;
  } else {
    std::cout
      << ("[ChAprilTagDetector::setSubPixelWindowSize] sub pixel window size should meet width==height and width > 3.");
  }
  return is_ok;
}

bool ChAprilTagDetector::detectChessCorners(const cv::Mat& _img_gray, cv::Mat& _img_dst) {
  bool is_ok = false;
  for (std::size_t i = 0; i < board_points_.size(); ++i) {
    const auto& board_info = board_infos_[board_type_index_[board_points_[i].board_type]];
    std::array<int, 2> odd_even_row_tags;
    if (board_info.layout.width & 0x1) {
      odd_even_row_tags.at(0) = board_info.layout.width / 2 + 1;
    } else {
      odd_even_row_tags.at(0) = board_info.layout.width / 2;
    }
    odd_even_row_tags.at(1) =
      board_info.layout.width - odd_even_row_tags.at(0);  // number of first row must >= second row

    std::array<std::size_t, 2> inlier_points_size = { std::size_t(board_info.layout.width - 1),
                                                      std::size_t(board_info.layout.height - 1) };
    int id_index                                  = 0;  // pattern1 +1 or new row
    for (std::size_t j = 0; j < inlier_points_size.at(1); ++j) {
      for (std::size_t k = 0; k < inlier_points_size.at(0); ++k) {
        std::size_t row_col_sum = j + k;
        cv::Point2d chess_corner(-1, -1);
        int tag_id = id_index + board_info.tag_range.at(0);
        if (row_col_sum & 0x1)  // pattern 2: top_right, bottom_left
        {
          int tag1_index = getTagIndex(board_points_[i].tags, tag_id);
          int tag2_index = getTagIndex(board_points_[i].tags, tag_id + odd_even_row_tags.at(1));
          if (tag1_index >= 0 && tag2_index >= 0) {
            chess_corner = getPattern2CornerPoint(board_points_[i].tags[tag1_index], board_points_[i].tags[tag2_index]);
          }
          if (k == inlier_points_size.at(0) - 1) {
            ++id_index;
          }
        } else  // pattern 1: top_left, bottom_right
        {
          int tag1_index = getTagIndex(board_points_[i].tags, tag_id);
          int tag2_index = getTagIndex(board_points_[i].tags, tag_id + odd_even_row_tags.at(0));
          if (tag1_index >= 0 && tag2_index >= 0) {
            chess_corner = getPattern1CornerPoint(board_points_[i].tags[tag1_index], board_points_[i].tags[tag2_index]);
          }
          ++id_index;
        }

        if (chess_corner.x > 0 && chess_corner.y > 0) {
          board_points_[i].chess_corners_3d.emplace_back(getChessCornerPoint3D(board_info.chess_len * 0.001f, j, k));
          board_points_[i].chess_corners_2d.emplace_back(chess_corner);
          board_points_[i].chess_corner_indices.push_back(j * inlier_points_size.at(0) + k);
        }
      }
    }
    if (board_points_[i].chess_corner_indices.size() > 0) {
      is_ok = true;
      getCornerSubPix(_img_gray, board_points_[i].chess_corners_2d);
      drawChessCorners(board_points_[i].chess_corners_2d, board_points_[i].chess_corner_indices, _img_dst);
    } else {
      std::cout << (std::to_string(i) + " board fail to detect chess corner.");
    }
  }
  return is_ok;
}

std::vector<ChAprilBoardPoints> ChAprilTagDetector::getChAprilBoardPoints() const {
  return board_points_;
}

std::vector<std::vector<cv::Point2d>> ChAprilTagDetector::getChessBoard2DCorners() const {
  std::vector<std::vector<cv::Point2d>> tmp;
  for (std::size_t i = 0; i < board_points_.size(); ++i) {
    tmp.push_back(board_points_[i].chess_corners_2d);
  }
  return tmp;
}

std::vector<std::vector<cv::Point3d>> ChAprilTagDetector::getChessBoard3DCorners() const {
  std::vector<std::vector<cv::Point3d>> tmp;
  for (std::size_t i = 0; i < board_points_.size(); ++i) {
    tmp.push_back(board_points_[i].chess_corners_3d);
  }
  return tmp;
}

std::vector<std::vector<int>> ChAprilTagDetector::getChessBoardCornerIndices() const {
  std::vector<std::vector<int>> indices;
  for (std::size_t i = 0; i < board_points_.size(); ++i) {
    indices.push_back(board_points_[i].chess_corner_indices);
  }
  return indices;
}

std::vector<std::vector<AprilTagInfo>> ChAprilTagDetector::getAprilTagInfo() const {
  std::vector<std::vector<AprilTagInfo>> tmp;
  for (std::size_t i = 0; i < board_points_.size(); ++i) {
    tmp.push_back(board_points_[i].tags);
  }
  return tmp;
}

std::vector<ChAprilBoardInfo> ChAprilTagDetector::getBoardInfo() const {
  return board_infos_;
}

cv::Point3d ChAprilTagDetector::getBoardCentroid(const int _board_index) const {
  cv::Point3d point(-1, -1, 0);
  if (_board_index >= 0 && std::size_t(_board_index) < board_infos_.size()) {
    point.x = 0.5 * (board_infos_[_board_index].layout.width - 2) * board_infos_[_board_index].chess_len * 0.001;
    point.y = 0.5 * (board_infos_[_board_index].layout.height - 2) * board_infos_[_board_index].chess_len * 0.001;
  } else {
    std::cout << ("ChAprilTagDetector::getBoardCentroid: invalid board index: " + std::to_string(_board_index));
  }
  return point;
}

void ChAprilTagDetector::printChAprilBoardPoints() const {
  for (const ChAprilBoardPoints& item : board_points_) {
    std::string blank(23, ' ');
    std::cout << ("board type: " + std::to_string(item.board_type));
    std::cout << ("family: " + item.family);

    std::string point2d_info =
      "There is " + std::to_string(item.chess_corners_2d.size()) + " chess_corners_2d_points :" + "\n";
    for (auto point2d : item.chess_corners_2d) {
      point2d_info += (blank + std::to_string(point2d.x) + "    " + std::to_string(point2d.y) + "\n");
    }
    std::cout << (point2d_info);

    std::string point3d_info =
      "There is " + std::to_string(item.chess_corners_3d.size()) + " chess_corners_3d_points :" + "\n";
    for (const auto& point3d : item.chess_corners_3d) {
      point3d_info += (blank + std::to_string(point3d.x) + "    " + std::to_string(point3d.y) + "    " +
                       std::to_string(point3d.z) + "\n");
    }
    std::cout << (point3d_info);

    for (const auto& tag : item.tags) {
      tag.print();
    }
    std::string tag_info = "There is " + std::to_string(item.tags.size()) + " tags :" + "\n";
    if (item.tags.size() < 3) {
      std::cout << (tag_info);
      return;
    }
  }
}

bool ChAprilTagDetector::setTagAreaThreshold(const std::map<std::string, double>& _tag_area_map) {
  bool is_ok = true;
  for (auto& board_info : board_infos_) {
    if (_tag_area_map.count(board_info.family) > 0) {
      board_info.tag_area_threshold = _tag_area_map.at(board_info.family);
    } else {
      is_ok = false;
      std::cout << (board_info.family + " tag area threshold doesn't exist.");
    }
  }
  return is_ok;
}
}  // namespace calib
}  // namespace robosense
