#include "roh_hand_rviz_panel/hand_panel.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>

namespace roh_hand_rviz_panel {

HandPanel::HandPanel(QWidget* parent)
    : rviz::Panel(parent), status_label_(nullptr) {
  gesture_pub_ = nh_.advertise<std_msgs::String>("/hand/gesture", 1);
  target_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("/hand/target", 1);

  auto* root = new QVBoxLayout;

  auto* presets = new QGroupBox("Presets");
  auto* preset_layout = new QGridLayout;
  auto* open_button = new QPushButton("Open");
  auto* close_button = new QPushButton("Close");
  auto* pinch2_button = new QPushButton("2-Finger Pinch");
  auto* pinch3_button = new QPushButton("3-Finger Pinch");
  auto* cylinder_button = new QPushButton("Cylinder");
  preset_layout->addWidget(open_button, 0, 0);
  preset_layout->addWidget(close_button, 0, 1);
  preset_layout->addWidget(pinch2_button, 1, 0);
  preset_layout->addWidget(pinch3_button, 1, 1);
  preset_layout->addWidget(cylinder_button, 2, 0, 1, 2);
  presets->setLayout(preset_layout);
  root->addWidget(presets);

  connect(open_button, &QPushButton::clicked, this, &HandPanel::sendOpen);
  connect(close_button, &QPushButton::clicked, this, &HandPanel::sendClose);
  connect(pinch2_button, &QPushButton::clicked, this, &HandPanel::sendPinch2);
  connect(pinch3_button, &QPushButton::clicked, this, &HandPanel::sendPinch3);
  connect(cylinder_button, &QPushButton::clicked, this, &HandPanel::sendCylinder);

  static const std::array<const char*, 6> kActuatorNames = {
      "Thumb Bend", "Index", "Middle", "Ring", "Little", "Thumb Rotate"};
  auto* actuators = new QGroupBox("Actuators");
  auto* actuator_layout = new QGridLayout;
  for (std::size_t i = 0; i < sliders_.size(); ++i) {
    auto* label = new QLabel(kActuatorNames[i]);
    sliders_[i] = new QSlider(Qt::Horizontal);
    spin_boxes_[i] = new QSpinBox;
    sliders_[i]->setRange(0, 1000);
    spin_boxes_[i]->setRange(0, 1000);
    actuator_layout->addWidget(label, static_cast<int>(i), 0);
    actuator_layout->addWidget(sliders_[i], static_cast<int>(i), 1);
    actuator_layout->addWidget(spin_boxes_[i], static_cast<int>(i), 2);
    connect(sliders_[i], &QSlider::valueChanged,
            spin_boxes_[i], &QSpinBox::setValue);
    connect(spin_boxes_[i], qOverload<int>(&QSpinBox::valueChanged),
            sliders_[i], &QSlider::setValue);
  }
  actuators->setLayout(actuator_layout);
  root->addWidget(actuators);

  auto* apply_button = new QPushButton("Apply Target");
  connect(apply_button, &QPushButton::clicked, this, &HandPanel::sendTarget);
  root->addWidget(apply_button);

  status_label_ = new QLabel("Ready");
  root->addWidget(status_label_);
  root->addStretch();
  setLayout(root);
}

void HandPanel::sendGesture(const char* name) {
  std_msgs::String msg;
  msg.data = name;
  gesture_pub_.publish(msg);
  setStatus(QString("Sent: %1").arg(name));
}

void HandPanel::sendOpen() { sendGesture("open"); }
void HandPanel::sendClose() { sendGesture("close"); }
void HandPanel::sendPinch2() { sendGesture("pinch_2"); }
void HandPanel::sendPinch3() { sendGesture("pinch_3"); }
void HandPanel::sendCylinder() { sendGesture("cylinder"); }

void HandPanel::sendTarget() {
  std_msgs::UInt16MultiArray msg;
  msg.data.reserve(sliders_.size());
  for (const auto* slider : sliders_) {
    msg.data.push_back(static_cast<uint16_t>(slider->value()));
  }
  target_pub_.publish(msg);
  setStatus("Target sent");
}

void HandPanel::setStatus(const QString& text) {
  status_label_->setText(text);
}

void HandPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  for (std::size_t i = 0; i < sliders_.size(); ++i) {
    config.mapSetValue(QString("actuator_%1").arg(i), sliders_[i]->value());
  }
}

void HandPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  for (std::size_t i = 0; i < sliders_.size(); ++i) {
    int value = 0;
    if (config.mapGetInt(QString("actuator_%1").arg(i), &value)) {
      sliders_[i]->setValue(value);
    }
  }
}

}  // namespace roh_hand_rviz_panel

PLUGINLIB_EXPORT_CLASS(roh_hand_rviz_panel::HandPanel, rviz::Panel)
