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

#ifndef CORNER_POINT_DETECTOR_H
#define CORNER_POINT_DETECTOR_H

#include "board_info.h"
#include <opencv2/core/mat.hpp>

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

struct apriltag_family;
struct apriltag_detector;
struct zarray;

namespace robosense
{
namespace calib
{
struct AprilTagInfo
{
  int id;
  int index;  // index on one board
  std::array<cv::Point2d, 4> corners_2d;
  std::array<cv::Point3d, 4> corners_3d;
  cv::Point2d getTagCenter() const
  {
    cv::Point2d center = corners_2d.at(0);
    for (std::size_t i = 1; i < corners_2d.size(); ++i)
    {
      center += corners_2d.at(i);
    }
    return center / static_cast<double>(corners_2d.size());
  }
  void print() const;
};

struct ChAprilBoardPoints
{
  int board_type;
  std::string family;
  std::vector<AprilTagInfo> tags;         // sorted based on id
  std::vector<int> chess_corner_indices;  // start from 0
  std::vector<cv::Point2d> chess_corners_2d;
  std::vector<cv::Point3d> chess_corners_3d;
};

template <typename T>
using DeletedUniquePtr = std::unique_ptr<T, std::function<void(T*)>>;

class ChAprilTagDetector
{
public:
  explicit ChAprilTagDetector();
  ~ChAprilTagDetector();
  /**
   * @brief Init board information before detect april tag.
   * 
   * @param _config_yaml config file
   * @return  true if init successfully, otherwise return false
   */
  bool init(const std::string& _config_yaml);

  /**
   * @brief Detect April Tag.
   * 
   * @param _img_src input image
   * @param _img_dst output image with detected information
   * @return true if detect april tag successfully, otherwise return false 
   */
  bool detect(const cv::Mat& _img_src, cv::Mat& _img_dst);

  /**
   * @brief Get the Detected Chess April Board Points object
   * 
   * @return std::vector<ChAprilBoardPoints>  all information (2D, 3D corner points, april tag info) of each board
   */
  std::vector<ChAprilBoardPoints> getChAprilBoardPoints() const;

  /**
   * @brief Print the Chess April Board Points information (2D, 3D corner points, april tag info) of each board
   */
  void printChAprilBoardPoints() const;

  /**
   * @brief Get the Detected Chess Board 2D Corners object
   * 
   * @return std::vector<std::vector<cv::Point2d>> detected 2D corner points of chess board
   */
  std::vector<std::vector<cv::Point2d>> getChessBoard2DCorners() const;

  /**
   * @brief Get the Detected Chess Board 3D Corners object
   * 
   * @return std::vector<std::vector<cv::Point3d>> detected 3D corner points of chess board
   */
  std::vector<std::vector<cv::Point3d>> getChessBoard3DCorners() const;

  /**
   * @brief Get the Detected Chess Board Corner Indices
   * 
   * @return std::vector<std::vector<int>> 
   */
  std::vector<std::vector<int>> getChessBoardCornerIndices() const;

  /**
   * @brief Get the Detected April Tag Info object
   * 
   * @return std::vector<std::vector<AprilTagInfo>> info of april tag
   */
  std::vector<std::vector<AprilTagInfo>> getAprilTagInfo() const;

  /**
   * @brief Get the Detected Board Info object
   * 
   * @return std::vector<ChAprilBoardInfo> all boards information
   */
  std::vector<ChAprilBoardInfo> getBoardInfo() const;

  /**
   * @brief Get the Board Centroid object
   * 
   * @param _board_index the index of board in config yaml file
   * @return cv::Point3d centroid point, x, y < 0 if board index is invalid
   */
  cv::Point3d getBoardCentroid(const int _board_index) const;

  /**
   * @brief Set the Sub Pixel Window Size object. 
   * 
   * @param _size Half of the side length of the search window. For example, if _size=Size(5,5), then a (2*5+1, 2*5+1) search window is used.
   * @return true if window size is valid, otherwise return false 
   */
  bool setSubPixelWindowSize(const cv::Size& _size);

  /**
   * @brief Convert std::vector<cv::Point2f> to std::vector<cv::Point2d>
   * 
   * @param _pf input point2f vector
   * @param _pd output point2d vector
   */
  void convertPoint2fTo2d(const std::vector<cv::Point2f>& _pf, std::vector<cv::Point2d>& _pd);

  /**
   * @brief Convert std::vector<cv::Point2d> to std::vector<cv::Point2f>
   * 
   * @param _pd input point2d vector
   * @param _pf output point2f vector
   */
  void convertPoint2dTo2f(const std::vector<cv::Point2d>& _pd, std::vector<cv::Point2f>& _pf);

  /**
   * @brief Set the Tag Area Threshold object
   * 
   * @param _tag_area_map tag area threshold of each family
   * @return true if set successfully, otherwise return false 
   */
  bool setTagAreaThreshold(const std::map<std::string, double>& _tag_area_map);

private:
  bool readBoardInfo(const std::string& _config_yaml);
  bool checkAprilTagValid();
  bool detectChessCorners(const cv::Mat& _img_gray, cv::Mat& _img_dst);
  DeletedUniquePtr<apriltag_family> creatAprilTagFamily(const std::string& _family_name);
  DeletedUniquePtr<apriltag_detector> creatAprilTagDetector(apriltag_family* _family);
  void parseTags(const zarray* _detections, cv::Mat& _img);
  int getTagIndex(const std::vector<AprilTagInfo>& _vec, const int _tag);
  cv::Point2d getPattern1CornerPoint(const AprilTagInfo& _top_left, const AprilTagInfo& _bottom_right);
  cv::Point2d getPattern2CornerPoint(const AprilTagInfo& _top_right, const AprilTagInfo& _bottom_left);
  cv::Point3d getChessCornerPoint3D(const double _chess_len, const std::size_t _row, const std::size_t _col);

  std::array<cv::Point3d, 4> getTagCorners3D(const ChAprilBoardInfo& _board_info, const int _tag_id);
  void getCornerSubPix(const cv::Mat& _img_gray, std::vector<cv::Point2d>& _point);
  std::string makeBoardId(const ChAprilBoardInfo& _board_info);
  std::map<int, std::array<cv::Point3d, 4>> initBoardTagCorners3D(const ChAprilBoardInfo& _board_info);

  std::vector<ChAprilBoardInfo> board_infos_;
  std::vector<ChAprilBoardPoints> board_points_;
  std::vector<DeletedUniquePtr<apriltag_family>> families_;
  std::vector<DeletedUniquePtr<apriltag_detector>> tag_detectors_;
  std::map<int, int> board_type_index_;
  std::map<std::string, std::map<int, std::array<cv::Point3d, 4>>> layout_board_tag_3d_corners_;
  cv::Size sub_pixel_window_size_;
};  // class ChAprilTagDetector

void drawApriltags(const std::vector<AprilTagInfo>& _tags, cv::Mat& _image);
void drawApriltags(const std::vector<ChAprilBoardPoints>& _board_points, cv::Mat& _image);
void drawChessCorners(const std::vector<cv::Point2d>& _point, const std::vector<int>& _corner_index, cv::Mat& _img_dst);
}  // namespace calib
}  // namespace robosense
#endif  // CORNER_POINT_DETECTOR_H
