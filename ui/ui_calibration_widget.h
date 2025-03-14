/********************************************************************************
** Form generated from reading UI file 'calibration_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATION_WIDGET_H
#define UI_CALIBRATION_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CalibrationWidget {
public:
  QGridLayout* gridLayout_3;
  QWidget* widget;
  QGridLayout* gridLayout;
  QGridLayout* gridLayout_2;
  QGridLayout* gridLayout_config;
  QFrame* frame_config;
  QFrame* frame_3;
  QGridLayout* gridLayout_4;

  QLineEdit* cfg_path_input_;
  QPushButton* btn_load_cfg_;
  QPushButton* btn_start_drive_;
  QPushButton* btn_camera_int_;
  QPushButton* btn_ext_;
  QPushButton* btn_start_calib_;
  QTextEdit* textedit_output_;

  void setupUi(QWidget* CalibrationWidget) {
    if (CalibrationWidget->objectName().isEmpty())
      CalibrationWidget->setObjectName(QString::fromUtf8("CalibrationWidget"));
    CalibrationWidget->resize(760, 468);
    gridLayout_3 = new QGridLayout(CalibrationWidget);
    gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
    widget = new QWidget(CalibrationWidget);
    widget->setObjectName(QString::fromUtf8("widget"));
    gridLayout = new QGridLayout(widget);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    gridLayout->setContentsMargins(0, 0, 0, 0);

    textedit_output_ = new QTextEdit(widget);
    textedit_output_->setObjectName(QString::fromUtf8("textedit_output_"));
    textedit_output_->setReadOnly(true);
    gridLayout->addWidget(textedit_output_, 1, 0, 1, 1);

    gridLayout_2 = new QGridLayout();
    gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));

    frame_config = new QFrame(widget);
    frame_config->setFixedHeight(50);  // 设置固定高度为 200 像素
    frame_config->setObjectName(QString::fromUtf8("frame_config"));
    frame_config->setFrameShape(QFrame::StyledPanel);
    frame_config->setFrameShadow(QFrame::Raised);
    gridLayout_config = new QGridLayout(frame_config);
    gridLayout_config->setObjectName(QString::fromUtf8("gridLayout_config"));

    QLabel* config_path_label = new QLabel("config yaml: ", frame_config);
    config_path_label->setObjectName(QString::fromUtf8("config_path_label"));
    gridLayout_config->addWidget(config_path_label, 0, 0, 1, 1);

    cfg_path_input_ = new QLineEdit(frame_config);
    cfg_path_input_->setObjectName(QString::fromUtf8("cfg_path_input_"));
    gridLayout_config->addWidget(cfg_path_input_, 0, 1, 1, 1);

    btn_load_cfg_ = new QPushButton(frame_config);
    btn_load_cfg_->setObjectName(QString::fromUtf8("btn_load_cfg_"));
    gridLayout_config->addWidget(btn_load_cfg_, 0, 2, 1, 1);

    frame_3 = new QFrame(widget);
    frame_3->setObjectName(QString::fromUtf8("frame_3"));
    frame_3->setFrameShape(QFrame::StyledPanel);
    frame_3->setFrameShadow(QFrame::Raised);
    gridLayout_4 = new QGridLayout(frame_3);
    gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));

    btn_start_drive_ = new QPushButton(frame_3);
    btn_start_drive_->setObjectName(QString::fromUtf8("btn_start_drive_"));
    gridLayout_4->addWidget(btn_start_drive_, 0, 0, 1, 1);

    btn_camera_int_ = new QPushButton(frame_3);
    btn_camera_int_->setObjectName(QString::fromUtf8("btn_camera_int_"));
    gridLayout_4->addWidget(btn_camera_int_, 0, 1, 1, 1);

    btn_ext_ = new QPushButton(frame_3);
    btn_ext_->setObjectName(QString::fromUtf8("btn_ext_"));
    gridLayout_4->addWidget(btn_ext_, 1, 0, 1, 1);

    btn_start_calib_ = new QPushButton(frame_3);
    btn_start_calib_->setObjectName(QString::fromUtf8("btn_start_calib_"));
    gridLayout_4->addWidget(btn_start_calib_, 1, 1, 1, 1);

    gridLayout_2->addWidget(frame_3, 1, 0, 1, 1);

    gridLayout_2->setRowStretch(0, 1);
    gridLayout_2->setRowStretch(1, 1);
    gridLayout_2->setRowStretch(2, 1);

    gridLayout->addLayout(gridLayout_2, 1, 1, 1, 1);
    gridLayout->addWidget(frame_config, 0, 0, 1, 2);

    gridLayout->setColumnStretch(0, 1);
    gridLayout->setColumnStretch(1, 1);

    gridLayout_3->addWidget(widget, 0, 0, 1, 1);

    retranslateUi(CalibrationWidget);

    QMetaObject::connectSlotsByName(CalibrationWidget);
  }  // setupUi

  void retranslateUi(QWidget* CalibrationWidget) {
    CalibrationWidget->setWindowTitle(QCoreApplication::translate("CalibrationWidget", "Form", nullptr));
    btn_load_cfg_->setText(QCoreApplication::translate("CalibrationWidget", "open", nullptr));
    btn_start_drive_->setText(QCoreApplication::translate("CalibrationWidget", "Start Driver", nullptr));
    btn_camera_int_->setText(QCoreApplication::translate("CalibrationWidget", "Camera Int", nullptr));
    btn_ext_->setText(QCoreApplication::translate("CalibrationWidget", "Camera-Lidar", nullptr));
    btn_start_calib_->setText(QCoreApplication::translate("CalibrationWidget", "Camera-Imu", nullptr));
    textedit_output_->setHtml(QCoreApplication::translate(
      "CalibrationWidget",
      "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
      "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
      "p, li { white-space: pre-wrap; }\n"
      "</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
      "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; "
      "text-indent:0px;\"></p></body></html>",
      nullptr));
  }  // retranslateUi
};

namespace Ui {
class CalibrationWidget : public Ui_CalibrationWidget {};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // UI_CALIBRATION_WIDGET_H
