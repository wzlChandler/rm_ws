#pragma once

#include <array>
#include <cstdint>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/panel.h>

class QLabel;
class QSlider;
class QSpinBox;
class QString;

namespace roh_hand_rviz_panel {

class HandPanel : public rviz::Panel {
  Q_OBJECT

public:
  explicit HandPanel(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void sendOpen();
  void sendClose();
  void sendPinch2();
  void sendPinch3();
  void sendCylinder();
  void sendTarget();

private:
  void sendGesture(const char* name);
  void setStatus(const QString& text);

  ros::NodeHandle nh_;
  ros::Publisher gesture_pub_;
  ros::Publisher target_pub_;
  std::array<QSlider*, 6> sliders_;
  std::array<QSpinBox*, 6> spin_boxes_;
  QLabel* status_label_;
};

}  // namespace roh_hand_rviz_panel
