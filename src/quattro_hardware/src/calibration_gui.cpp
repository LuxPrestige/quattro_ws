// Standalone Qt calibration GUI. Deliberately independent of ros2_control/
// controller_manager (docs/packages/quattro_hardware.md section 5) -- it
// talks to gim6010_driver directly and must not run at the same time as
// hardware.launch.py (docs/calibration.md safety condition).
//
// Always drives motors via MIT (cmd 0x08) regardless of the runtime
// hardware_control_method, using each joint's calibration.yaml kp/kd as the
// MIT hold gain -- this is a fixed property of the calibration procedure
// itself (docs/calibration.md), not of whatever control method the robot
// will run afterward.

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "gim6010_driver/motor_manager.hpp"
#include "quattro_hardware/joint_transform.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kJogStepRad = kPi / 180.0;  // 1 degree
constexpr float kCalibrationVelocityLimitRevS = 5.0F;
constexpr float kCalibrationCurrentLimitA = 10.0F;
constexpr double kFixedGearRatio = 8.0;

// Fixed hardware wiring/mounting facts (docs/packages/quattro_hardware.md
// section 0) -- not user-configurable. A calibration.yaml that disagrees
// with this table describes a different (or mis-wired) robot and is
// rejected rather than trusted.
struct CanonicalJoint
{
  const char * name;
  uint8_t can_id;
  const char * bus;
  double direction;
};

constexpr std::array<CanonicalJoint, 12> kCanonicalJoints = {
  {{"front_left_hip_joint", 0, "can0", -1.0},
    {"front_left_upper_leg_joint", 1, "can0", -1.0},
    {"front_left_lower_leg_joint", 2, "can0", -1.0},
    {"front_right_hip_joint", 3, "can0", -1.0},
    {"front_right_upper_leg_joint", 4, "can0", 1.0},
    {"front_right_lower_leg_joint", 5, "can0", 1.0},
    {"back_left_hip_joint", 6, "can1", 1.0},
    {"back_left_upper_leg_joint", 7, "can1", -1.0},
    {"back_left_lower_leg_joint", 8, "can1", -1.0},
    {"back_right_hip_joint", 9, "can1", 1.0},
    {"back_right_upper_leg_joint", 10, "can1", 1.0},
    {"back_right_lower_leg_joint", 11, "can1", 1.0}}};

struct CalibratedJoint
{
  std::string name;
  std::string can_bus;
  uint8_t can_id{0};
  double direction{1.0};
  double offset{0.0};
  double current_limit{5.0};
  double kp{0.0};
  double kd{0.0};
};

std::vector<CalibratedJoint> load_calibration(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node joints_node = root["joints"];
  if (!joints_node) {
    throw std::runtime_error("calibration file has no top-level 'joints' key: " + path);
  }

  std::vector<CalibratedJoint> result;
  for (const auto & canonical : kCanonicalJoints) {
    const YAML::Node joint_node = joints_node[canonical.name];
    if (!joint_node) {
      throw std::runtime_error(
        std::string("calibration file is missing joint '") + canonical.name + "'");
    }

    CalibratedJoint joint;
    joint.name = canonical.name;
    joint.can_bus = joint_node["can_interface"].as<std::string>();
    joint.can_id = static_cast<uint8_t>(joint_node["can_id"].as<int>());
    joint.direction = joint_node["direction"].as<double>();
    joint.offset = joint_node["offset"].as<double>();
    joint.current_limit = joint_node["current_limit"] ? joint_node["current_limit"].as<double>() : 5.0;
    joint.kp = joint_node["kp"].as<double>();
    joint.kd = joint_node["kd"].as<double>();

    if (joint.can_bus != canonical.bus || joint.can_id != canonical.can_id ||
      joint.direction != canonical.direction)
    {
      throw std::runtime_error(
        "joint '" + joint.name + "' can_interface/can_id/direction does not match the canonical "
        "mapping (docs/packages/quattro_hardware.md section 0) -- refusing to load a file that "
        "may describe the wrong robot");
    }
    if (joint.kp < 0.0 || joint.kp > gim6010_driver::kMitKpMax || joint.kd < 0.0 ||
      joint.kd > gim6010_driver::kMitKdMax)
    {
      throw std::runtime_error(
        "joint '" + joint.name + "' kp/kd is outside the GIM6010 MIT range");
    }

    result.push_back(joint);
  }
  return result;
}

void save_calibration(const std::string & path, const std::vector<CalibratedJoint> & joints)
{
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node joints_node = root["joints"];
  for (const auto & joint : joints) {
    YAML::Node joint_node = joints_node[joint.name];
    joint_node["can_interface"] = joint.can_bus;
    joint_node["can_id"] = static_cast<int>(joint.can_id);
    joint_node["direction"] = static_cast<int>(joint.direction);
    joint_node["offset"] = joint.offset;
    joint_node["current_limit"] = joint.current_limit;
    joint_node["kp"] = joint.kp;
    joint_node["kd"] = joint.kd;
  }

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out) {
      throw std::runtime_error("failed to open temp file for writing: " + tmp_path);
    }
    out << root;
  }
  if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace calibration file: " + path);
  }
}

}  // namespace

class CalibrationWindow : public QWidget
{
public:
  CalibrationWindow(std::string calibration_file, std::vector<CalibratedJoint> joints)
  : calibration_file_(std::move(calibration_file)), joints_(std::move(joints))
  {
    enabled_.assign(joints_.size(), false);
    target_joint_rad_.assign(joints_.size(), 0.0);

    std::vector<gim6010_driver::MotorRoute> routes;
    std::set<std::string> seen_buses;
    for (const auto & joint : joints_) {
      if (seen_buses.insert(joint.can_bus).second) {
        buses_.push_back(joint.can_bus);
      }
      routes.push_back(gim6010_driver::MotorRoute{joint.can_id, joint.can_bus});
    }
    motor_manager_ = std::make_unique<gim6010_driver::MotorManager>(buses_, routes);
    if (!motor_manager_->open()) {
      QMessageBox::critical(this, "Calibration", "Failed to open one or more CAN buses");
    }

    build_ui();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CalibrationWindow::on_timer_tick);
    timer_->start(50);

    selected_index_ = joints_.empty() ? -1 : 0;
    update_status_label();
  }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (enabled_[i]) {
        idle_motor(static_cast<int>(i));
      }
    }
    if (motor_manager_) {
      motor_manager_->close();
    }
    QWidget::closeEvent(event);
  }

private:
  void build_ui()
  {
    setWindowTitle("Quattro Calibration");

    joint_selector_ = new QComboBox(this);
    for (const auto & joint : joints_) {
      joint_selector_->addItem(QString::fromStdString(joint.name));
    }
    connect(
      joint_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
      &CalibrationWindow::on_selection_changed);

    enable_selected_btn_ = new QPushButton("Enable Selected Motor", this);
    disable_selected_btn_ = new QPushButton("Disable Selected Motor", this);
    enable_all_btn_ = new QPushButton("Enable All Motors", this);
    disable_all_btn_ = new QPushButton("Disable All Motors", this);
    minus_btn_ = new QPushButton("-1 deg", this);
    plus_btn_ = new QPushButton("+1 deg", this);
    save_btn_ = new QPushButton("Save Current Position as Zero", this);
    status_label_ = new QLabel(this);
    status_label_->setWordWrap(true);

    connect(enable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_selected);
    connect(
      disable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_selected);
    connect(enable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_all);
    connect(disable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_all);
    connect(minus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_minus);
    connect(plus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_plus);
    connect(save_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_save_zero);

    auto * layout = new QVBoxLayout(this);
    layout->addWidget(
      new QLabel(QString("Calibration file: %1").arg(QString::fromStdString(calibration_file_))));
    layout->addWidget(joint_selector_);

    auto * single_group = new QGroupBox("Single motor", this);
    auto * single_layout = new QHBoxLayout(single_group);
    single_layout->addWidget(enable_selected_btn_);
    single_layout->addWidget(disable_selected_btn_);
    layout->addWidget(single_group);

    auto * all_group = new QGroupBox("All motors (hold all, jog selected)", this);
    auto * all_layout = new QHBoxLayout(all_group);
    all_layout->addWidget(enable_all_btn_);
    all_layout->addWidget(disable_all_btn_);
    layout->addWidget(all_group);

    auto * jog_layout = new QHBoxLayout();
    jog_layout->addWidget(minus_btn_);
    jog_layout->addWidget(plus_btn_);
    layout->addLayout(jog_layout);

    layout->addWidget(save_btn_);
    layout->addWidget(status_label_);
  }

  quattro_hardware::JointCalibration session_calibration(const CalibratedJoint & joint) const
  {
    // offset is always 0 within a live session: the session's own joint_rad
    // coordinate is "direction-corrected position relative to wherever the
    // motor happened to be when it was enabled." The real offset is only
    // computed (and written to the file) in on_save_zero().
    return quattro_hardware::JointCalibration{joint.direction, 0.0, kFixedGearRatio};
  }

  bool request_fresh_motor_rev(int index, double & out_motor_rev)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->request_encoder_estimate(joint.can_id);
    for (int attempt = 0; attempt < 20; ++attempt) {
      QThread::msleep(10);
      motor_manager_->poll();
      if (auto * motor = motor_manager_->motor(joint.can_id)) {
        if (const auto estimate = motor->last_encoder_estimate()) {
          out_motor_rev = estimate->position_rev;
          return true;
        }
      }
    }
    return false;
  }

  void send_hold_command(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    const auto calibration = session_calibration(joint);
    gim6010_driver::MitCommand command;
    command.position_rad =
      quattro_hardware::joint_rad_to_mit_output_rad(target_joint_rad_[static_cast<size_t>(index)], calibration);
    command.velocity_rad_s = 0.0;
    command.kp = joint.kp;
    command.kd = joint.kd;
    command.torque_Nm = 0.0;
    if (!motor_manager_->send_mit_command(joint.can_id, command)) {
      status_label_->setText(
        QString("WARNING: %1 hold command rejected (outside MIT protocol range)")
        .arg(QString::fromStdString(joint.name)));
    }
  }

  void set_motor_closed_loop(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->send_set_limits(joint.can_id, kCalibrationVelocityLimitRevS, kCalibrationCurrentLimitA);

    double motor_rev = 0.0;
    if (!request_fresh_motor_rev(index, motor_rev)) {
      QMessageBox::warning(
        this, "Calibration",
        QString("No encoder feedback from '%1' -- not enabling").arg(QString::fromStdString(joint.name)));
      return;
    }
    target_joint_rad_[static_cast<size_t>(index)] =
      quattro_hardware::motor_rev_to_joint_rad(motor_rev, session_calibration(joint));

    // Deliberately no Clear_Errors here, same as QuattroSystem: a
    // pre-existing fault must stay visible, not get silently wiped by
    // enabling.
    motor_manager_->send_set_controller_mode(
      joint.can_id, gim6010_driver::ControlMode::kPositionControl,
      gim6010_driver::InputMode::kMitMotionControl);
    motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kClosedLoopControl);
    enabled_[static_cast<size_t>(index)] = true;
    send_hold_command(index);
  }

  void idle_motor(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kIdle);
    enabled_[static_cast<size_t>(index)] = false;
  }

  void update_status_label()
  {
    if (selected_index_ < 0) {
      status_label_->setText("No joint selected.");
      return;
    }
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    const double target_deg = target_joint_rad_[static_cast<size_t>(selected_index_)] * 180.0 / kPi;
    const char * mode_text = mode_ == Mode::kSingle ? "single" : (mode_ == Mode::kAll ? "all" : "none");
    status_label_->setText(
      QString("Selected: %1  |  enabled: %2  |  mode: %3  |  target: %4 deg  |  saved offset: %5 rad")
      .arg(QString::fromStdString(joint.name))
      .arg(enabled_[static_cast<size_t>(selected_index_)] ? "yes" : "no")
      .arg(mode_text)
      .arg(target_deg, 0, 'f', 3)
      .arg(joint.offset, 0, 'f', 6));
  }

  void on_selection_changed(int index)
  {
    if (mode_ == Mode::kSingle && selected_index_ >= 0 &&
      selected_index_ != index && enabled_[static_cast<size_t>(selected_index_)])
    {
      idle_motor(selected_index_);
      mode_ = Mode::kNone;
    }
    selected_index_ = index;
    update_status_label();
  }

  void on_enable_selected()
  {
    if (selected_index_ < 0) {
      return;
    }
    if (mode_ == Mode::kAll) {
      QMessageBox::warning(this, "Calibration", "Disable All Motors first.");
      return;
    }
    if (mode_ == Mode::kSingle) {
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (enabled_[i]) {
          idle_motor(static_cast<int>(i));
        }
      }
    }
    set_motor_closed_loop(selected_index_);
    mode_ = Mode::kSingle;
    update_status_label();
  }

  void on_disable_selected()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    idle_motor(selected_index_);
    if (mode_ == Mode::kSingle) {
      mode_ = Mode::kNone;
    }
    update_status_label();
  }

  void on_enable_all()
  {
    if (mode_ == Mode::kSingle) {
      QMessageBox::warning(this, "Calibration", "Disable Selected Motor first.");
      return;
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      set_motor_closed_loop(static_cast<int>(i));
    }
    mode_ = Mode::kAll;
    if (selected_index_ < 0 && !joints_.empty()) {
      selected_index_ = 0;
    }
    update_status_label();
  }

  void on_disable_all()
  {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (enabled_[i]) {
        idle_motor(static_cast<int>(i));
      }
    }
    mode_ = Mode::kNone;
    update_status_label();
  }

  void on_jog_minus()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    target_joint_rad_[static_cast<size_t>(selected_index_)] -= kJogStepRad;
    send_hold_command(selected_index_);
    update_status_label();
  }

  void on_jog_plus()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    target_joint_rad_[static_cast<size_t>(selected_index_)] += kJogStepRad;
    send_hold_command(selected_index_);
    update_status_label();
  }

  void on_save_zero()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      QMessageBox::warning(this, "Calibration", "Select and enable a motor first.");
      return;
    }
    auto & joint = joints_[static_cast<size_t>(selected_index_)];

    double motor_rev = 0.0;
    if (!request_fresh_motor_rev(selected_index_, motor_rev)) {
      QMessageBox::warning(this, "Calibration", "No fresh encoder feedback -- not saving.");
      return;
    }
    joint.offset = quattro_hardware::motor_rev_to_joint_rad(motor_rev, session_calibration(joint));

    try {
      save_calibration(calibration_file_, joints_);
    } catch (const std::exception & error) {
      QMessageBox::critical(
        this, "Calibration", QString("Failed to save calibration file: %1").arg(error.what()));
      return;
    }

    if (mode_ == Mode::kSingle) {
      idle_motor(selected_index_);
      mode_ = Mode::kNone;
    }
    // In "all" mode the joint stays enabled holding its just-saved zero so
    // the operator can move on to the next joint (docs/calibration.md).

    update_status_label();
    QMessageBox::information(
      this, "Calibration",
      QString("Saved offset for %1: %2 rad")
      .arg(QString::fromStdString(joint.name)).arg(joint.offset, 0, 'f', 6));
  }

  void on_timer_tick()
  {
    motor_manager_->poll();
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (enabled_[i]) {
        send_hold_command(static_cast<int>(i));
      }
    }
  }

  enum class Mode { kNone, kSingle, kAll };

  std::string calibration_file_;
  std::vector<CalibratedJoint> joints_;
  std::vector<std::string> buses_;
  std::unique_ptr<gim6010_driver::MotorManager> motor_manager_;

  std::vector<bool> enabled_;
  std::vector<double> target_joint_rad_;
  Mode mode_{Mode::kNone};
  int selected_index_{-1};

  QComboBox * joint_selector_{nullptr};
  QPushButton * enable_selected_btn_{nullptr};
  QPushButton * disable_selected_btn_{nullptr};
  QPushButton * enable_all_btn_{nullptr};
  QPushButton * disable_all_btn_{nullptr};
  QPushButton * minus_btn_{nullptr};
  QPushButton * plus_btn_{nullptr};
  QPushButton * save_btn_{nullptr};
  QLabel * status_label_{nullptr};
  QTimer * timer_{nullptr};
};

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  std::string calibration_file;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--calibration-file" && i + 1 < argc) {
      calibration_file = argv[++i];
    }
  }
  if (calibration_file.empty()) {
    std::fprintf(stderr, "Usage: calibration_gui --calibration-file <path>\n");
    return 1;
  }

  std::vector<CalibratedJoint> joints;
  try {
    joints = load_calibration(calibration_file);
  } catch (const std::exception & error) {
    QMessageBox::critical(
      nullptr, "Calibration", QString("Failed to load calibration file: %1").arg(error.what()));
    return 1;
  }

  CalibrationWindow window(calibration_file, std::move(joints));
  window.show();
  return app.exec();
}
