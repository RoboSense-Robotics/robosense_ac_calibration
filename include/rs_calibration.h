#pragma once

#include "calib_core.h"

#include <QSettings>
#include <image_transport/image_transport.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Ui {
class CalibrationWidget;
}

namespace rs_calibration_plugin {

class RSCalibration : public rviz_common::Panel {
  Q_OBJECT
public:
  RSCalibration(QWidget* parent = 0);
  ~RSCalibration() override;

private slots:

  void startDriver();
  void cameraIntrinsicsMode();
  void ExtrinsicsMode();
  void calibrate();
  void loadConfig();

public slots:
  void updateCalibOutput(const QString& qstr);

signals:
  void calibOutputUpdated(const QString& qstr);

protected:
  void onInitialize() override;

  void loadConfig(const QString& file_name);

  void initROSNode();

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void publishPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);

  void showBoardIndicator(const sensor_msgs::msg::Image::SharedPtr msg);
  void showCalibProcess();

  void autoCalibProcess();
  bool autoCaptureImage();

  void setupLayout();
  void setupActions();

  void initBoxGeometry();
  void publishBoardPose();
  void updateSensorPose();

  void calibrateCameraIntrinsics();
  void calibrateExtrinsics();

private:
  QSettings settings_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_detect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_indicate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr board_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_pose_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_board_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fit_board_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pro_board_pub_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_msg_queue_;
  std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_queue_;

  std::shared_ptr<visualization_msgs::msg::Marker> board_box_ptr_;
  std::shared_ptr<visualization_msgs::msg::Marker> sensor_box_ptr_;

  std::string image_sub_topic_;
  std::string cloud_sub_topic_;
  std::string imu_sub_topic_;

  Ui::CalibrationWidget* ui_;
  std::shared_ptr<CalibCore> calib_ptr_ = nullptr;

  std::thread auto_calibrating_thread_;
  std::atomic<bool> auto_calibrating_;

  CalibStatus calib_status_ = CalibStatus::None;
};

}  // namespace rs_calibration_plugin