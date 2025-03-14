#include "rs_calibration.h"

#include "ui_calibration_widget.h"
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

namespace rs_calibration_plugin {
RSCalibration::RSCalibration(QWidget* parent) :
  rviz_common::Panel(parent), ui_(new Ui::CalibrationWidget), settings_("RobosenseSDK", "Calibration") {
  node_ = std::make_shared<rclcpp::Node>("rs_calibration");
  initBoxGeometry();
}

RSCalibration::~RSCalibration() {

  auto_calibrating_ = false;
  if (auto_calibrating_thread_.joinable()) {
    auto_calibrating_thread_.join();
  }

  rclcpp::shutdown();
  node_.reset();
}

void RSCalibration::onInitialize() {
  setupLayout();
  setupActions();
}

void RSCalibration::setupLayout() {
  // 创建布局和控件
  QVBoxLayout* layout = new QVBoxLayout;
  QWidget* widget     = new QWidget;

  ui_->setupUi(widget);
  layout->addWidget(widget);
  setLayout(layout);

  // 暂时关闭Camera-Imu标定功能
  ui_->btn_start_calib_->setEnabled(false);

  QString last_config_path = settings_.value("last_config_path", "").toString();
  std::cout << last_config_path.toStdString() << std::endl;

  // 如果之前保存了路径，直接加载配置文件
  if (!last_config_path.isEmpty() && QFile::exists(last_config_path)) {
    ui_->cfg_path_input_->setText(last_config_path);
    loadConfig(last_config_path);
    return;
  }
}

void RSCalibration::setupActions() {
  QObject::connect(ui_->btn_load_cfg_, SIGNAL(clicked()), this, SLOT(loadConfig()));

  QObject::connect(ui_->btn_start_drive_, SIGNAL(clicked()), this, SLOT(startDriver()));
  QObject::connect(ui_->btn_camera_int_, SIGNAL(clicked()), this, SLOT(cameraIntrinsicsMode()));
  QObject::connect(ui_->btn_ext_, SIGNAL(clicked()), this, SLOT(ExtrinsicsMode()));
  QObject::connect(ui_->btn_start_calib_, SIGNAL(clicked()), this, SLOT(calibrate()));

  QObject::connect(this, &RSCalibration::calibOutputUpdated, this, &RSCalibration::updateCalibOutput);
}

void RSCalibration::initBoxGeometry() {

  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = "rslidar";
  marker_msg.header.stamp    = rclcpp::Clock().now();

  marker_msg.ns     = "box_pose";
  marker_msg.id     = 0;
  marker_msg.type   = visualization_msgs::msg::Marker::CUBE;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;

  board_box_ptr_  = std::make_shared<visualization_msgs::msg::Marker>(marker_msg);
  sensor_box_ptr_ = std::make_shared<visualization_msgs::msg::Marker>(marker_msg);

  board_box_ptr_->pose.position.x    = 0.0;
  board_box_ptr_->pose.position.y    = 0.0;
  board_box_ptr_->pose.position.z    = 1.0;
  board_box_ptr_->pose.orientation.x = 0.0;
  board_box_ptr_->pose.orientation.y = std::sqrt(2.0) / 2;
  board_box_ptr_->pose.orientation.z = 0.0;
  board_box_ptr_->pose.orientation.w = std::sqrt(2.0) / 2;

  board_box_ptr_->color.r = 1.0;
  board_box_ptr_->color.g = 0.0;
  board_box_ptr_->color.b = 0.0;
  board_box_ptr_->color.a = 1.0;

  board_box_ptr_->scale.x = 0.6;
  board_box_ptr_->scale.y = 0.6;
  board_box_ptr_->scale.z = 0.01;

  sensor_box_ptr_->pose.position.x    = 1.0;
  sensor_box_ptr_->pose.position.y    = 0.0;
  sensor_box_ptr_->pose.position.z    = 1.0;
  sensor_box_ptr_->pose.orientation.x = 0.0;
  sensor_box_ptr_->pose.orientation.y = std::sqrt(2.0) / 2;
  sensor_box_ptr_->pose.orientation.z = 0.0;
  sensor_box_ptr_->pose.orientation.w = std::sqrt(2.0) / 2;

  sensor_box_ptr_->color.r = 0.0;
  sensor_box_ptr_->color.g = 1.0;
  sensor_box_ptr_->color.b = 0.0;
  sensor_box_ptr_->color.a = 1.0;

  sensor_box_ptr_->scale.x = 0.1;
  sensor_box_ptr_->scale.y = 0.1;
  sensor_box_ptr_->scale.z = 0.05;
}

void RSCalibration::initROSNode() {

  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_sub_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) { this->imageCallback(msg); });
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_sub_topic_, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->cloudCallback(msg); });
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_sub_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imuCallback(msg); });

  img_detect_pub_   = node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_detecte", 10);
  img_indicate_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_indicate", 10);

  raw_board_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_board_points", 10);
  fit_board_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/fit_board_points", 10);
  pro_board_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/pro_board_points", 10);

  board_pose_pub_  = node_->create_publisher<visualization_msgs::msg::Marker>("/board_pose", 10);
  sensor_pose_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/sensor_pose", 10);
}

void RSCalibration::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // return;
  img_msg_queue_.push(msg);
  if (img_msg_queue_.size() > 1) {
    img_msg_queue_.pop();
  }

  if (calib_ptr_ && auto_calibrating_ && calib_status_ == CalibStatus::CameraInt) {
    showBoardIndicator(msg);
  }
}

void RSCalibration::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (calib_status_ < CalibStatus::Ext) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *cloud_ptr);

  cloud_queue_.push(cloud_ptr);
  if (cloud_queue_.size() > 1) {
    cloud_queue_.pop();
  }
  return;
}

void RSCalibration::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (calib_status_ < CalibStatus::Ext) {
    return;
  }
  calib_ptr_->setImuMsgInput(msg);
  return;
}

void RSCalibration::publishPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                                      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
  if (cloud_ptr == nullptr || cloud_ptr->empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2);

  pcl::toROSMsg(*cloud_ptr, *cloud_msg);
  cloud_msg->header.stamp    = rclcpp::Clock().now();
  cloud_msg->header.frame_id = "rslidar";
  cloud_msg->height          = 1;
  cloud_msg->width           = cloud_ptr->size();

  pub->publish(*cloud_msg);
}

void RSCalibration::loadConfig(const QString& file_name) {
  std::string config_file = file_name.toStdString();
  if (access(config_file.c_str(), F_OK) == -1) {
    RCLCPP_INFO(node_->get_logger(), "config file does not exist!!! %s", config_file);
    loadConfig();
  }

  ui_->cfg_path_input_->setText(file_name);

  YAML::Node doc = YAML::LoadFile(config_file);

  image_sub_topic_ = doc["image_topic"].as<std::string>();
  cloud_sub_topic_ = doc["cloud_topic"].as<std::string>();
  imu_sub_topic_   = doc["imu_topic"].as<std::string>();

  calib_ptr_.reset(new CalibCore);
  calib_ptr_->setConfigFile(config_file);

  // 保存选择的路径到设置中
  settings_.setValue("last_config_path", file_name);
  RCLCPP_INFO(node_->get_logger(), "save config path: %s", file_name);
}

void RSCalibration::loadConfig() {

  // 如果没有保存路径，弹出文件对话框让用户选择
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Config file (*.yaml *.yml)"));
  if (file_name.endsWith(".yaml") || file_name.endsWith(".yml")) {
    loadConfig(file_name);
  } else {
    int ret = QMessageBox::warning(this, tr("Warning"), tr("Please choose one config file!"),
                                   QMessageBox::Cancel | QMessageBox::Ok);
    if (ret == QMessageBox::Ok) {
      loadConfig();
    }
  }
}

void RSCalibration::startDriver() {

  if (calib_ptr_ == nullptr) {
    loadConfig();
  }

  ui_->btn_start_drive_->setEnabled(false);
  std::cout << "init ROS2 Node" << std::endl;

  std::string str = "";
  str += "Please choose the calibration type\n\n";
  str += "请选择标定类型\n";
  emit calibOutputUpdated(QString::fromStdString(str));

  initROSNode();

  std::thread([this]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();
  }).detach();

  // 启动自动标定线程
  auto_calibrating_        = true;
  auto_calibrating_thread_ = std::thread(&RSCalibration::autoCalibProcess, this);

  return;
}

void RSCalibration::cameraIntrinsicsMode() {
  calib_ptr_->initCameraConfig();
  calib_status_ = CalibStatus::CameraInt;
  ui_->btn_camera_int_->setEnabled(false);
  ui_->btn_ext_->setEnabled(true);
  return;
  /*
  std::cout << "img_msg_queue size: " << img_msg_queue_.size();
  if (img_msg_queue_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "img_msg_queue is empty");
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "img_msg_queue size: %zu", img_msg_queue_.size());

  auto img_msg = img_msg_queue_.back();
  if (!img_msg || img_msg->data.empty()) {
    std::cerr << "Received an invalid image message!" << std::endl;
    return;
  }

  std::cout << "Image width: " << img_msg->width << ", height: " << img_msg->height << std::endl;
  std::cout << "Image encoding: " << img_msg->encoding << std::endl;
  std::cout << "Image step: " << img_msg->step << std::endl;
  std::cout << "Expected step: " << img_msg->width * 3 << std::endl;  // assuming 3 channels (BGR)

  if (img_msg->step < img_msg->width * 3) {
    return;
  }

  auto cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
  auto img    = cv_ptr->image;

  calib_ptr_->pushImage(img, img);

  if (img.type() != CV_8UC3) {
    RCLCPP_ERROR(node_->get_logger(), "Input image type is not CV_8UC3, but %d", img.type());
  }

  sensor_msgs::msg::Image::SharedPtr dst_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), img_msg->encoding, img).toImageMsg();
  img_detect_pub_->publish(*dst_msg);

  showCalibProcess();
  return;
  */
}

void RSCalibration::ExtrinsicsMode() {
  calib_status_ = CalibStatus::Ext;
  ui_->btn_camera_int_->setEnabled(true);
  ui_->btn_ext_->setEnabled(false);
  return;
}

void RSCalibration::calibrate() {
  return;
  std::string str = "calibrating camera intrinsics...please wait";
  emit calibOutputUpdated(QString::fromStdString(str));

  calib_ptr_->startCalib(calib_status_);

  str = calib_ptr_->getCalibResult();
  emit calibOutputUpdated(QString::fromStdString(str));

  auto_calibrating_ = false;
  ui_->btn_start_drive_->setEnabled(true);
  ui_->btn_camera_int_->setEnabled(true);
}

void RSCalibration::updateCalibOutput(const QString& qstr) {
  ui_->textedit_output_->setPlainText(qstr);
}

void RSCalibration::updateSensorPose() {
  auto img_cnt         = calib_ptr_->getImageCount();
  size_t current_pose  = img_cnt / 6;
  size_t current_index = img_cnt % 6;
  switch (current_pose) {
  case 0:
    sensor_box_ptr_->pose.position.x = 1.0;
    sensor_box_ptr_->pose.position.y = 0.0;
    sensor_box_ptr_->pose.position.z = 1.0;
    break;
  case 1:  // left
    sensor_box_ptr_->pose.position.x = std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.y = -std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.z = 1.0;
    break;
  case 2:
    sensor_box_ptr_->pose.position.x = std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.y = std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.z = 1.0;
    break;
  case 3:
    sensor_box_ptr_->pose.position.x = std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.y = 0.0;
    sensor_box_ptr_->pose.position.z = 1.0 + std::sqrt(2.0) / 2;
    break;
  case 4:
    sensor_box_ptr_->pose.position.x = std::sqrt(2.0) / 2;
    sensor_box_ptr_->pose.position.y = 0.0;
    sensor_box_ptr_->pose.position.z = 1.0 - std::sqrt(2.0) / 2;
    break;
  default: break;
  }

  // publishBoardPose();
}

void RSCalibration::showBoardIndicator(const sensor_msgs::msg::Image::SharedPtr raw_msg) {

  auto img_msg = raw_msg;
  if (!img_msg || img_msg->data.empty()) {
    std::cerr << "Received an invalid image message!" << std::endl;
    return;
  }

  if (img_msg->step < img_msg->width * 3) {
    img_msg->step = img_msg->width * 3;
  }

  auto cv_ptr  = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
  auto raw_img = cv_ptr->image;
  // cv::Mat raw_img = robosense::FromImageMsg(*img_msg);
  calib_ptr_->addBoardIndicator(raw_img);

  sensor_msgs::msg::Image::SharedPtr indicate_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), img_msg->encoding, raw_img).toImageMsg();
  // sensor_msgs::msg::Image::SharedPtr indicate_msg = robosense::ToImageMsg(raw_img);
  img_indicate_pub_->publish(*indicate_msg);
}

void RSCalibration::showCalibProcess() {

  auto img_cnt = calib_ptr_->getImageCount();

  size_t current_pose          = img_cnt / 6;
  size_t current_index         = img_cnt % 6;
  std::string pose_description = "";

  switch (current_pose) {
  case 0:
    pose_description += "Please position the camera directly facing the calibration board, maintaining a frontal view";
    break;
  case 1:
    pose_description += "Please move the camera to the left side of the calibration board, maintaining a lateral view";
    break;
  case 2:
    pose_description += "Please move the camera to the right side of the calibration board, maintaining a lateral view";
    break;
  case 3: pose_description += "Please move the camera above the calibration board, maintaining an overhead view"; break;
  case 4: pose_description += "Please move the camera below the calibration board, maintaining an upward view"; break;
  default: pose_description += "Calibration is complete"; break;
  }

  pose_description += "\n\n";

  switch (current_pose) {
  case 0: pose_description += "请将相机正对靶板，保持正视靶板"; break;
  case 1: pose_description += "请将相机移动到靶板左侧，保持侧视靶板"; break;
  case 2: pose_description += "请将相机移动到靶板右侧，保持侧视靶板"; break;
  case 3: pose_description += "请将相机移动到靶板上侧，保持俯视靶板"; break;
  case 4: pose_description += "请将相机移动到靶板下侧，保持仰视靶板"; break;
  default: pose_description += "标定已完成"; break;
  }

  std::string str = pose_description + "\n\n";
  str += "current image count: " + std::to_string(current_index) + "/6";
  // ui_->textedit_output_->setPlainText(QString::fromStdString(str));
  emit calibOutputUpdated(QString::fromStdString(str));
}

void RSCalibration::publishBoardPose() {

  board_pose_pub_->publish(*board_box_ptr_);
  sensor_pose_pub_->publish(*sensor_box_ptr_);
}

void RSCalibration::calibrateCameraIntrinsics() {
  updateSensorPose();
  if (autoCaptureImage()) {
    updateSensorPose();
  }
  if (calib_ptr_->getImageCount() < 30) {
    return;
  }

  std::string str = "calibrating camera intrinsics...please wait";
  emit calibOutputUpdated(QString::fromStdString(str));

  calib_ptr_->startCalib(calib_status_);

  str = calib_ptr_->getCalibResult();
  emit calibOutputUpdated(QString::fromStdString(str));

  auto_calibrating_ = false;
  ui_->btn_start_drive_->setEnabled(true);

  return;
}

void RSCalibration::calibrateExtrinsics() {
  if (!calib_ptr_->isExtCalibReady() || img_msg_queue_.empty()) {
    return;
  }
  if (calib_ptr_->getExtCalibPose() >= 1) {
    std::cout << "calibrateExtrinsics" << std::endl;
    calib_ptr_->startCalib(calib_status_);
    std::cout << "finish calibrate extrinsics" << std::endl;
    ui_->btn_ext_->setEnabled(true);
    calib_status_ = CalibStatus::None;

    auto str = calib_ptr_->getCalibResult();
    emit calibOutputUpdated(QString::fromStdString(str));
    return;
  }

  calib_ptr_->setImageMsgInput(img_msg_queue_.back());
  calib_ptr_->setPointcloudInput(cloud_queue_.back());
  calib_ptr_->addExtCalibPose();
  calib_ptr_->setExtCalibReady(false);
}

void RSCalibration::autoCalibProcess() {
  while (auto_calibrating_) {

    switch (calib_status_) {
    case CalibStatus::None: break;
    case CalibStatus::CameraInt: calibrateCameraIntrinsics(); break;
    case CalibStatus::Ext: calibrateExtrinsics(); break;
    default: break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

bool RSCalibration::autoCaptureImage() {
  if (img_msg_queue_.empty()) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "img_msg_queue size: %zu", img_msg_queue_.size());

  auto img_msg = img_msg_queue_.back();

  if (img_msg->step < img_msg->width * 3) {
    img_msg->step = img_msg->width * 3;
  }

  cv::Mat img = cv_bridge::toCvCopy(img_msg, img_msg->encoding)->image;
  // cv::Mat img = robosense::FromImageMsg(*img_msg);
  if (calib_ptr_->autoPushImage(img)) {
    sensor_msgs::msg::Image::SharedPtr dst_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), img_msg->encoding, img).toImageMsg();
    // sensor_msgs::msg::Image::SharedPtr dst_msg = robosense::ToImageMsg(img);
    img_detect_pub_->publish(*dst_msg);
    showCalibProcess();
    return true;
  }
  return false;
}

}  // namespace rs_calibration_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rs_calibration_plugin::RSCalibration, rviz_common::Panel)