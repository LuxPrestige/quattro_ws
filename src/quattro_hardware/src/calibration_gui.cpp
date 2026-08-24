// Standalone Qt calibration + position-control tuning GUI. Deliberately
// independent of ros2_control/controller_manager (docs/packages/
// quattro_hardware.md section 5) -- it talks to gim6010_driver directly and
// must not run at the same time as hardware.launch.py (docs/calibration.md
// safety condition). This used to be two separate binaries
// (calibration_gui + position_control_tuning_gui); the tuning GUI has been
// folded in here so offset calibration and gain tuning can be done in one
// session without tearing down and re-opening the CAN buses.
//
// Uses the same Position Control + Pos Filter startup path as runtime
// bringup: limits and gains are applied, then the controller mode, then
// closed-loop control -- which by itself holds the axis where it is. No
// Set_Input_Pos is sent when a motor is enabled; the first one is only sent
// when the operator asks for actual movement (jog, absolute target, or Move
// to Saved Zero).
//
// All 12 axes are shown at once in a table (state, saved/session/target
// angle, tracking error, velocity, raw motor reading, offset) so a faulted
// or uncalibrated axis is visible without having to select it first.

#include <QApplication>
#include <QAbstractItemView>
#include <QBrush>
#include <QCloseEvent>
#include <QColor>
#include <QDoubleValidator>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollBar>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "gim6010_driver/motor_manager.hpp"
#include "quattro_hardware/joint_transform.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kCalibrationVelocityLimitRevS = 5.0F;
constexpr double kFixedGearRatio = 8.0;
// The motors broadcast Get_Encoder_Estimates (0x09) on their own, so this
// GUI never requests one -- it only judges whether what has already arrived
// is recent enough to act on. Generous relative to the broadcast period so
// a few dropped frames do not read as "no feedback".
constexpr std::chrono::milliseconds kFeedbackTimeout{100};
// Heartbeat is broadcast at 100ms, so this tolerates three misses before
// the axis-state indicator gives up and says so.
constexpr std::chrono::milliseconds kHeartbeatTimeout{400};

// Explicit foreground *and* background on the state cell: the Qt palette
// this runs under is not known ahead of time, and a bare coloured
// foreground can end up unreadable on a dark theme. These are the only
// colours in the GUI, so they are spelled out here rather than in a
// stylesheet file.
const QColor kClosedLoopBgColor(0x1b, 0x7f, 0x3b);
const QColor kIdleBgColor(0xb3, 0x26, 0x1e);
const QColor kUnknownBgColor(0x6b, 0x6b, 0x6b);

// Table column layout. Joint/CAN/Offset are fixed per row; the rest are
// refreshed every timer tick from live feedback.
enum Column
{
  kColJoint = 0,
  kColCan,
  kColState,
  kColSaved,
  kColSession,
  kColTarget,
  kColError,
  kColVel,
  kColMotor,
  kColOffset,
  kColCount
};

// Fixed hardware wiring/mounting facts (docs/packages/quattro_hardware.md
// section 0) -- not user-configurable. A calibration.yaml that disagrees
// with this table describes a different (or mis-wired) robot and is
// rejected rather than trusted. lower_rad/upper_rad are a copy of the
// <limit> tags in quattro.urdf.xacro; if the two ever disagree, the URDF is
// the source of truth and this table should be updated to match it.
struct CanonicalJoint
{
  const char * name;
  uint8_t can_id;
  const char * bus;
  double direction;
  double lower_rad;
  double upper_rad;
};

constexpr std::array<CanonicalJoint, 12> kCanonicalJoints = {
  {{"front_left_hip_joint", 0, "can0", -1.0, -1.04, 1.04},
    {"front_left_upper_leg_joint", 1, "can0", -1.0, -1.57079632679, 2.59},
    {"front_left_lower_leg_joint", 2, "can0", -1.0, -2.4783675378, 2.4783675378},
    {"front_right_hip_joint", 3, "can0", -1.0, -1.04, 1.04},
    {"front_right_upper_leg_joint", 4, "can0", 1.0, -1.57079632679, 2.59},
    {"front_right_lower_leg_joint", 5, "can0", 1.0, -2.4783675378, 2.4783675378},
    {"back_left_hip_joint", 6, "can1", 1.0, -1.04, 1.04},
    {"back_left_upper_leg_joint", 7, "can1", -1.0, -1.57079632679, 2.59},
    {"back_left_lower_leg_joint", 8, "can1", -1.0, -2.4783675378, 2.4783675378},
    {"back_right_hip_joint", 9, "can1", 1.0, -1.04, 1.04},
    {"back_right_upper_leg_joint", 10, "can1", 1.0, -1.57079632679, 2.59},
    {"back_right_lower_leg_joint", 11, "can1", 1.0, -2.4783675378, 2.4783675378}}};

struct CalibratedJoint
{
  std::string name;
  std::string can_bus;
  uint8_t can_id{0};
  double direction{1.0};
  double offset{0.0};
  double lower_rad{0.0};
  double upper_rad{0.0};
};

struct CalibrationData
{
  double current_limit{0.0};
  double position_gain{0.0};
  double velocity_gain{0.0};
  double velocity_integrator_gain{0.0};
  std::vector<CalibratedJoint> joints;
};

bool gains_valid(
  double current_limit, double position_gain, double velocity_gain,
  double velocity_integrator_gain)
{
  return std::isfinite(current_limit) && std::isfinite(position_gain) &&
         std::isfinite(velocity_gain) && std::isfinite(velocity_integrator_gain) &&
         current_limit > 0.0 && position_gain >= 0.0 && velocity_gain >= 0.0 &&
         velocity_integrator_gain >= 0.0;
}

CalibrationData load_calibration(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node position_control = root["position_control"];
  if (!position_control) {
    throw std::runtime_error("calibration file has no top-level 'position_control' key: " + path);
  }
  const YAML::Node joints_node = root["joints"];
  if (!joints_node) {
    throw std::runtime_error("calibration file has no top-level 'joints' key: " + path);
  }

  CalibrationData result;
  result.current_limit = position_control["current_limit"].as<double>();
  result.position_gain = position_control["position_gain"].as<double>();
  result.velocity_gain = position_control["velocity_gain"].as<double>();
  result.velocity_integrator_gain = position_control["velocity_integrator_gain"].as<double>();
  if (!gains_valid(
      result.current_limit, result.position_gain, result.velocity_gain,
      result.velocity_integrator_gain))
  {
    throw std::runtime_error(
      "position_control current_limit must be positive and gains must be non-negative");
  }
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

    if (joint.can_bus != canonical.bus || joint.can_id != canonical.can_id ||
      joint.direction != canonical.direction)
    {
      throw std::runtime_error(
        "joint '" + joint.name + "' can_interface/can_id/direction does not match the canonical "
        "mapping (docs/packages/quattro_hardware.md section 0) -- refusing to load a file that "
        "may describe the wrong robot");
    }
    joint.lower_rad = canonical.lower_rad;
    joint.upper_rad = canonical.upper_rad;
    result.joints.push_back(joint);
  }
  return result;
}

// Writes only the `joints` key (can_interface/can_id/direction/offset).
// Kept separate from save_position_control() so that saving a zero offset
// never has a side effect of silently persisting whatever is currently
// typed (and possibly unvalidated) into the gain fields, and vice versa.
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

// Writes only the `position_control` key. See save_calibration() for why
// this is a separate function rather than one that writes everything.
void save_position_control(
  const std::string & path, double current_limit, double position_gain, double velocity_gain,
  double velocity_integrator_gain)
{
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node position_control = root["position_control"];
  position_control["current_limit"] = current_limit;
  position_control["position_gain"] = position_gain;
  position_control["velocity_gain"] = velocity_gain;
  position_control["velocity_integrator_gain"] = velocity_integrator_gain;

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
  CalibrationWindow(std::string calibration_file, CalibrationData calibration)
  : calibration_file_(std::move(calibration_file)),
    current_limit_(calibration.current_limit),
    position_gain_(calibration.position_gain),
    velocity_gain_(calibration.velocity_gain),
    velocity_integrator_gain_(calibration.velocity_integrator_gain),
    joints_(std::move(calibration.joints))
  {
    enabled_.assign(joints_.size(), false);
    has_target_.assign(joints_.size(), false);
    sync_sequence_.assign(joints_.size(), 0);
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
    if (!joints_.empty()) {
      table_->selectRow(0);
    }

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CalibrationWindow::on_timer_tick);
    timer_->start(50);

    update_status_label();
    update_table();
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

    reload_btn_ = new QPushButton("Reload Calibration from File", this);
    connect(reload_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_reload_from_file);

    table_ = new QTableWidget(static_cast<int>(joints_.size()), static_cast<int>(kColCount), this);
    const QStringList headers = {
      "Joint", "CAN", "State", "Saved [deg]", "Session [deg]", "Target [deg]",
      "Error [deg]", "Vel [deg/s]", "Motor [rev]", "Offset [rad]"};
    table_->setHorizontalHeaderLabels(headers);
    table_->verticalHeader()->setVisible(false);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    // Fixed-width digits: this table is refreshed every tick, and a
    // proportional font makes the numbers jitter sideways as they change,
    // which is exactly the wrong behaviour for a readout watched while
    // jogging a joint by hand.
    table_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));

    table_items_.resize(joints_.size());
    for (size_t row = 0; row < joints_.size(); ++row) {
      table_items_[row].resize(static_cast<size_t>(kColCount));
      for (int col = 0; col < kColCount; ++col) {
        auto * item = new QTableWidgetItem();
        if (col != kColJoint && col != kColCan) {
          item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        }
        table_->setItem(static_cast<int>(row), col, item);
        table_items_[row][static_cast<size_t>(col)] = item;
      }
      table_items_[row][kColJoint]->setText(QString::fromStdString(joints_[row].name));
      table_items_[row][kColCan]->setText(
        QString("%1:%2")
        .arg(QString::fromStdString(joints_[row].can_bus))
        .arg(joints_[row].can_id));
    }
    // Seed the live-updated columns with a worst-case-width string purely so
    // resizeColumnsToContents() below sizes every column (and therefore the
    // table's, and in turn the whole window's, initial sizeHint) wide enough
    // that opening the GUI never requires a manual resize to see a value in
    // full. Real ticks always overwrite this text with "--" or a
    // same-or-narrower formatted number (see update_row()), so column widths
    // stay put afterwards -- consistent with the fixed-width font choice
    // above, which exists for the same "no sideways jitter" reason.
    if (!table_items_.empty()) {
      auto & sizing_row = table_items_.front();
      sizing_row[kColState]->setText("CLOSED LOOP  fault 0x00000000");
      sizing_row[kColSaved]->setText("-999.999");
      sizing_row[kColSession]->setText("-999.999");
      sizing_row[kColTarget]->setText("-999.999");
      sizing_row[kColError]->setText("-999.999");
      sizing_row[kColVel]->setText("-9999.99");
      sizing_row[kColMotor]->setText("-999.999999");
      sizing_row[kColOffset]->setText("-9.999999");
    }
    table_->resizeColumnsToContents();
    // QTableWidget's sizeHint() ignores its actual column widths (it falls
    // back to QAbstractScrollArea's generic ~256x192 default), so without
    // this the resizeColumnsToContents() above has no effect on the
    // top-level window's initial size and the operator has to drag the
    // window wider by hand every time to see the rightmost columns. Setting
    // an explicit minimum width forces the surrounding QVBoxLayout -- and
    // therefore the window on first show() -- to actually be wide enough
    // for every column at once. The vertical scrollbar's width is included
    // because 12 rows of group boxes below the table can push the window
    // past screen height, which makes Qt add that scrollbar and would
    // otherwise eat into the last column.
    table_->setMinimumWidth(
      table_->horizontalHeader()->length() + table_->verticalHeader()->width() +
      table_->frameWidth() * 2 + table_->verticalScrollBar()->sizeHint().width() + 4);
    connect(
      table_, &QTableWidget::itemSelectionChanged, this,
      &CalibrationWindow::on_table_selection_changed);

    enable_selected_btn_ = new QPushButton("Enable Selected Motor", this);
    disable_selected_btn_ = new QPushButton("Disable Selected Motor", this);
    enable_all_btn_ = new QPushButton("Enable All Motors", this);
    disable_all_btn_ = new QPushButton("Disable All Motors", this);

    current_limit_edit_ = make_number(QString::number(current_limit_, 'g', 10), 0.001, 100.0);
    position_gain_edit_ = make_number(QString::number(position_gain_, 'g', 10), 0.0, 100000.0);
    velocity_gain_edit_ = make_number(QString::number(velocity_gain_, 'g', 10), 0.0, 100000.0);
    velocity_integrator_gain_edit_ = make_number(
      QString::number(velocity_integrator_gain_, 'g', 10), 0.0, 100000.0);
    apply_gains_btn_ = new QPushButton("Apply to Enabled Motors", this);
    save_gains_btn_ = new QPushButton("Save Gains to YAML", this);

    jog_step_deg_edit_ = make_number("1.0", 0.0001, 45.0);
    minus_btn_ = new QPushButton("- step", this);
    plus_btn_ = new QPushButton("+ step", this);
    absolute_target_deg_edit_ = make_number("0.0", -360.0, 360.0);
    go_to_target_btn_ = new QPushButton("Go to Target", this);
    move_to_saved_zero_btn_ = new QPushButton("Move to Saved Zero", this);

    save_zero_btn_ = new QPushButton("Save Current Position as Zero", this);
    status_label_ = new QLabel(this);
    status_label_->setWordWrap(true);

    connect(
      enable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_selected);
    connect(
      disable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_selected);
    connect(enable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_all);
    connect(disable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_all);
    connect(apply_gains_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_apply_gains);
    connect(save_gains_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_save_gains);
    connect(minus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_minus);
    connect(plus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_plus);
    connect(go_to_target_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_go_to_target);
    connect(
      move_to_saved_zero_btn_, &QPushButton::clicked, this,
      &CalibrationWindow::on_move_to_saved_zero);
    connect(save_zero_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_save_zero);

    auto * layout = new QVBoxLayout(this);
    layout->addWidget(
      new QLabel(QString("Calibration file: %1").arg(QString::fromStdString(calibration_file_))));
    layout->addWidget(reload_btn_);
    layout->addWidget(table_);

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

    auto * gains_group = new QGroupBox("Position control gains (all motors)", this);
    auto * gains_form = new QFormLayout(gains_group);
    gains_form->addRow("Current limit (A)", current_limit_edit_);
    gains_form->addRow("Position gain", position_gain_edit_);
    gains_form->addRow("Velocity gain", velocity_gain_edit_);
    gains_form->addRow("Velocity integrator gain", velocity_integrator_gain_edit_);
    auto * gains_buttons = new QHBoxLayout();
    gains_buttons->addWidget(apply_gains_btn_);
    gains_buttons->addWidget(save_gains_btn_);
    gains_form->addRow(gains_buttons);
    layout->addWidget(gains_group);

    auto * motion_group = new QGroupBox("Motion (selected motor)", this);
    auto * motion_layout = new QVBoxLayout(motion_group);
    auto * jog_form = new QFormLayout();
    jog_form->addRow("Jog step (deg)", jog_step_deg_edit_);
    motion_layout->addLayout(jog_form);
    auto * jog_buttons = new QHBoxLayout();
    jog_buttons->addWidget(minus_btn_);
    jog_buttons->addWidget(plus_btn_);
    motion_layout->addLayout(jog_buttons);
    auto * target_form = new QFormLayout();
    target_form->addRow("Absolute target (deg, saved frame)", absolute_target_deg_edit_);
    motion_layout->addLayout(target_form);
    auto * target_buttons = new QHBoxLayout();
    target_buttons->addWidget(go_to_target_btn_);
    target_buttons->addWidget(move_to_saved_zero_btn_);
    motion_layout->addLayout(target_buttons);
    layout->addWidget(motion_group);

    layout->addWidget(save_zero_btn_);
    layout->addWidget(status_label_);
  }

  QLineEdit * make_number(const QString & value, double minimum, double maximum)
  {
    auto * edit = new QLineEdit(value, this);
    edit->setValidator(new QDoubleValidator(minimum, maximum, 9, edit));
    return edit;
  }

  quattro_hardware::JointCalibration session_calibration(const CalibratedJoint & joint) const
  {
    // offset is always 0 within a live session: the session's own joint_rad
    // coordinate is "direction-corrected position relative to wherever the
    // motor happened to be when it was enabled." The real offset is only
    // computed (and written to the file) in on_save_zero().
    return quattro_hardware::JointCalibration{joint.direction, 0.0, kFixedGearRatio};
  }

  // Blocks until a Get_Encoder_Estimates frame that arrived strictly after
  // this joint was synchronized to closed-loop control lands, or ~200ms
  // elapses. Nothing is requested: the motors broadcast 0x09 by themselves.
  //
  // The sequence comparison, not just freshness, is what makes the result
  // usable: on the GIM6010-8 a position sampled before the axis reached
  // closed-loop control can be garbage, and a recent timestamp says nothing
  // about which side of that transition the frame came from. Only a joint
  // that has already been enabled has a meaningful sync_sequence_, so this
  // is only valid to call on an enabled joint.
  bool read_post_closed_loop_motor_rev(int index, double & out_motor_rev)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    const std::uint64_t baseline = sync_sequence_[static_cast<size_t>(index)];
    for (int attempt = 0; attempt < 20; ++attempt) {
      motor_manager_->poll();
      const auto * motor = motor_manager_->motor(joint.can_id);
      if (motor != nullptr && motor->encoder_sequence() > baseline &&
        motor->has_fresh_feedback(kFeedbackTimeout, std::chrono::steady_clock::now()))
      {
        if (const auto reading = motor->last_encoder_estimate()) {
          out_motor_rev = reading->position_rev;
          return true;
        }
      }
      QThread::msleep(10);
    }
    return false;
  }

  // The non-blocking, freshness-only half, used by the live table. The
  // display is allowed to show a pre-closed-loop reading -- seeing what the
  // motor reports while idle is useful -- but nothing that is saved or
  // commanded goes through this path.
  std::optional<gim6010_driver::EncoderEstimate> fresh_motor_reading(
    const CalibratedJoint & joint) const
  {
    const auto * motor = motor_manager_->motor(joint.can_id);
    if (motor == nullptr ||
      !motor->has_fresh_feedback(kFeedbackTimeout, std::chrono::steady_clock::now()))
    {
      return std::nullopt;
    }
    return motor->last_encoder_estimate();
  }

  // Sends the joint's current target as a Set_Input_Pos. Only ever reached
  // once the operator has actually asked this joint to move: enabling a
  // motor does not call this (see enable_motor()).
  void send_target_command(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    const auto calibration = session_calibration(joint);
    gim6010_driver::SetInputPosCommand command;
    command.position_rev = static_cast<float>(quattro_hardware::joint_rad_to_motor_rev(
      target_joint_rad_[static_cast<size_t>(index)], calibration));
    if (!motor_manager_->send_set_input_pos(joint.can_id, command)) {
      status_label_->setText(
        QString("WARNING: %1 position command rejected")
        .arg(QString::fromStdString(joint.name)));
    }
  }

  // Marks a joint as having an operator-requested target and sends it. The
  // first call for a joint is also the first Set_Input_Pos that joint has
  // seen this session. target_joint_rad is always in the session frame
  // (session_calibration(), offset 0).
  void request_target(int index, double target_joint_rad)
  {
    target_joint_rad_[static_cast<size_t>(index)] = target_joint_rad;
    has_target_[static_cast<size_t>(index)] = true;
    send_target_command(index);
  }

  // Blocks until this motor's Heartbeat reports closed-loop control with no
  // axis error, or ~500ms elapses. Get_Error (0x03) goes unanswered on this
  // firmware, so Heartbeat is the only fault source available.
  bool wait_for_closed_loop(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    for (int attempt = 0; attempt < 50; ++attempt) {
      motor_manager_->poll();
      const auto * motor = motor_manager_->motor(joint.can_id);
      if (motor != nullptr &&
        motor->has_fresh_heartbeat(kHeartbeatTimeout, std::chrono::steady_clock::now()))
      {
        const auto heartbeat = *motor->last_heartbeat();
        if (heartbeat.axis_error != 0) {
          return false;
        }
        if (heartbeat.axis_state == gim6010_driver::AxisState::kClosedLoopControl) {
          return true;
        }
      }
      QThread::msleep(10);
    }
    return false;
  }

  // Same startup sequence QuattroSystem uses (docs/calibration.md, "실기
  // activation 원칙"): limits, gains, Position Control + Pos Filter, then
  // closed loop. Closed-loop entry alone holds the axis where it is, so no
  // Set_Input_Pos is sent here -- and none can be, correctly, because the
  // position it would carry is only trustworthy after the transition this
  // is requesting.
  bool enable_motor(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->send_set_limits(
      joint.can_id, kCalibrationVelocityLimitRevS, static_cast<float>(current_limit_));
    motor_manager_->send_set_pos_gain(joint.can_id, static_cast<float>(position_gain_));
    motor_manager_->send_set_vel_gains(
      joint.can_id, static_cast<float>(velocity_gain_),
      static_cast<float>(velocity_integrator_gain_));

    // Deliberately no Clear_Errors here, same as QuattroSystem: a
    // pre-existing fault must stay visible, not get silently wiped by
    // enabling.
    motor_manager_->send_set_controller_mode(
      joint.can_id, gim6010_driver::ControlMode::kPositionControl,
      gim6010_driver::InputMode::kPosFilter);
    motor_manager_->send_set_axis_state(joint.can_id,
      gim6010_driver::AxisState::kClosedLoopControl);

    if (!wait_for_closed_loop(index)) {
      motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kIdle);
      QMessageBox::warning(
        this, "Calibration",
        QString("'%1' did not report closed-loop control (or reported a fault) -- left idle")
        .arg(QString::fromStdString(joint.name)));
      return false;
    }

    // Baseline for "post-closed-loop" is taken here, after the Heartbeat
    // confirmed the transition, so every frame counted from now on was
    // sampled while the axis was already in closed loop.
    const auto * motor = motor_manager_->motor(joint.can_id);
    sync_sequence_[static_cast<size_t>(index)] =
      motor != nullptr ? motor->encoder_sequence() : 0;
    enabled_[static_cast<size_t>(index)] = true;

    double motor_rev = 0.0;
    if (!read_post_closed_loop_motor_rev(index, motor_rev)) {
      enabled_[static_cast<size_t>(index)] = false;
      motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kIdle);
      QMessageBox::warning(
        this, "Calibration",
        QString("No encoder feedback from '%1' after closed loop -- left idle")
        .arg(QString::fromStdString(joint.name)));
      return false;
    }
    // Session position only. has_target_ stays false: the motor is holding
    // itself, and nothing has been commanded to it yet.
    target_joint_rad_[static_cast<size_t>(index)] =
      quattro_hardware::motor_rev_to_joint_rad(motor_rev, session_calibration(joint));
    has_target_[static_cast<size_t>(index)] = false;
    sync_sequence_[static_cast<size_t>(index)] =
      motor != nullptr ? motor->encoder_sequence() : 0;
    return true;
  }

  void idle_motor(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kIdle);
    enabled_[static_cast<size_t>(index)] = false;
    // A re-enabled joint must start from its own fresh post-closed-loop
    // sync again, not resume commanding the target it had before.
    has_target_[static_cast<size_t>(index)] = false;
  }

  // Resends limits/gains to every currently-enabled axis without touching
  // axis state or the hold target -- used by both "Apply to Enabled
  // Motors" and Reload so a gain change takes effect immediately instead of
  // only on the next Enable.
  void apply_gains_to_enabled_motors()
  {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!enabled_[i]) {
        continue;
      }
      const auto & joint = joints_[i];
      motor_manager_->send_set_limits(
        joint.can_id, kCalibrationVelocityLimitRevS, static_cast<float>(current_limit_));
      motor_manager_->send_set_pos_gain(joint.can_id, static_cast<float>(position_gain_));
      motor_manager_->send_set_vel_gains(
        joint.can_id, static_cast<float>(velocity_gain_),
        static_cast<float>(velocity_integrator_gain_));
    }
  }

  bool read_gain_inputs(
    double & current_limit, double & position_gain, double & velocity_gain,
    double & velocity_integrator_gain)
  {
    bool ok_current = false;
    bool ok_position = false;
    bool ok_velocity = false;
    bool ok_integrator = false;
    current_limit = current_limit_edit_->text().toDouble(&ok_current);
    position_gain = position_gain_edit_->text().toDouble(&ok_position);
    velocity_gain = velocity_gain_edit_->text().toDouble(&ok_velocity);
    velocity_integrator_gain = velocity_integrator_gain_edit_->text().toDouble(&ok_integrator);
    if (!ok_current || !ok_position || !ok_velocity || !ok_integrator ||
      !gains_valid(current_limit, position_gain, velocity_gain, velocity_integrator_gain))
    {
      QMessageBox::warning(this, "Calibration", "Enter valid finite gain values.");
      return false;
    }
    return true;
  }

  bool read_jog_step_rad(double & step_rad)
  {
    bool ok = false;
    const double degrees = jog_step_deg_edit_->text().toDouble(&ok);
    if (!ok || !std::isfinite(degrees) || degrees <= 0.0) {
      QMessageBox::warning(this, "Calibration", "Enter a valid positive jog step.");
      return false;
    }
    step_rad = degrees * kPi / 180.0;
    return true;
  }

  void update_status_label()
  {
    if (selected_index_ < 0) {
      status_label_->setText("No joint selected.");
      return;
    }
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    const double target_deg = target_joint_rad_[static_cast<size_t>(selected_index_)] * 180.0 / kPi;
    const bool commanded = has_target_[static_cast<size_t>(selected_index_)];
    status_label_->setText(
      QString("Selected: %1  |  enabled: %2  |  %3 %4 deg (session)  |  saved offset: %5 rad")
      .arg(QString::fromStdString(joint.name))
      .arg(enabled_[static_cast<size_t>(selected_index_)] ? "yes" : "no")
      .arg(commanded ? "target:" : "holding at:")
      .arg(target_deg, 0, 'f', 3)
      .arg(joint.offset, 0, 'f', 6));
  }

  void on_table_selection_changed()
  {
    // Selecting a different row only changes which joint the jog/target
    // controls and Enable/Disable Selected act on -- it does not touch any
    // other motor's enabled state, so several joints can be held enabled at
    // once while calibrating them one at a time (e.g. holding one leg fixed
    // as a reference while adjusting an adjacent joint).
    const int row = table_->currentRow();
    if (row < 0) {
      return;
    }
    selected_index_ = row;
    update_status_label();
  }

  void on_enable_selected()
  {
    if (selected_index_ < 0) {
      return;
    }
    enable_motor(selected_index_);
    update_status_label();
  }

  void on_disable_selected()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    idle_motor(selected_index_);
    update_status_label();
  }

  void on_enable_all()
  {
    for (size_t i = 0; i < joints_.size(); ++i) {
      enable_motor(static_cast<int>(i));
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
    update_status_label();
  }

  void on_apply_gains()
  {
    double current_limit = 0.0;
    double position_gain = 0.0;
    double velocity_gain = 0.0;
    double velocity_integrator_gain = 0.0;
    if (!read_gain_inputs(current_limit, position_gain, velocity_gain, velocity_integrator_gain)) {
      return;
    }
    current_limit_ = current_limit;
    position_gain_ = position_gain;
    velocity_gain_ = velocity_gain;
    velocity_integrator_gain_ = velocity_integrator_gain;
    apply_gains_to_enabled_motors();
    status_label_->setText("Applied gains to all enabled motors.");
  }

  void on_save_gains()
  {
    double current_limit = 0.0;
    double position_gain = 0.0;
    double velocity_gain = 0.0;
    double velocity_integrator_gain = 0.0;
    if (!read_gain_inputs(current_limit, position_gain, velocity_gain, velocity_integrator_gain)) {
      return;
    }
    try {
      save_position_control(
        calibration_file_, current_limit, position_gain, velocity_gain,
        velocity_integrator_gain);
    } catch (const std::exception & error) {
      QMessageBox::critical(
        this, "Calibration", QString("Failed to save calibration file: %1").arg(error.what()));
      return;
    }
    current_limit_ = current_limit;
    position_gain_ = position_gain;
    velocity_gain_ = velocity_gain;
    velocity_integrator_gain_ = velocity_integrator_gain;
    QMessageBox::information(this, "Calibration", "Saved position control gains to YAML.");
  }

  // The jog buttons are one of three places a Set_Input_Pos originates
  // (with Go to Target and Move to Saved Zero). Until one of them is
  // pressed the joint is held by the motor itself and no position command
  // has been sent. Jog deliberately does not check the URDF joint limit:
  // during calibration the offset is often still wrong or zero, so the
  // saved-frame limit is meaningless here -- jogging is exactly how the
  // offset gets found in the first place.
  void on_jog_minus()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    double step_rad = 0.0;
    if (!read_jog_step_rad(step_rad)) {
      return;
    }
    request_target(
      selected_index_, target_joint_rad_[static_cast<size_t>(selected_index_)] - step_rad);
    update_status_label();
  }

  void on_jog_plus()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      return;
    }
    double step_rad = 0.0;
    if (!read_jog_step_rad(step_rad)) {
      return;
    }
    request_target(
      selected_index_, target_joint_rad_[static_cast<size_t>(selected_index_)] + step_rad);
    update_status_label();
  }

  // Drives the selected joint to an absolute angle expressed in the saved
  // (offset-applied, ROS joint) frame -- unlike jog, this is checked
  // against the URDF joint limit, because an absolute command can send the
  // joint somewhere far from wherever it happens to be sitting.
  void on_go_to_target()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      QMessageBox::warning(this, "Calibration", "Select and enable a motor first.");
      return;
    }
    bool ok = false;
    const double saved_target_deg = absolute_target_deg_edit_->text().toDouble(&ok);
    if (!ok || !std::isfinite(saved_target_deg)) {
      QMessageBox::warning(this, "Calibration", "Enter a valid absolute target.");
      return;
    }
    const double saved_target_rad = saved_target_deg * kPi / 180.0;
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    if (saved_target_rad < joint.lower_rad || saved_target_rad > joint.upper_rad) {
      QMessageBox::warning(
        this, "Calibration", "Requested target is outside the URDF joint limit.");
      return;
    }
    // session_rad = saved_rad + offset (session_calibration() uses offset
    // 0, so session_rad == saved_rad exactly at the saved zero, which is
    // also what Move to Saved Zero relies on).
    request_target(selected_index_, saved_target_rad + joint.offset);
    update_status_label();
  }

  void on_move_to_saved_zero()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      QMessageBox::warning(this, "Calibration", "Select and enable a motor first.");
      return;
    }
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    // target_joint_rad_ is always expressed in this session's offset-0
    // frame (session_calibration()): joint_rad = session_rad - offset, so
    // session_rad == offset is exactly the point where the real calibrated
    // joint_rad is 0 -- the saved zero from calibration.yaml (or from the
    // last Reload Calibration from File). This physically drives the motor
    // there instead of just holding wherever it already was.
    request_target(selected_index_, joint.offset);
    update_status_label();
  }

  void on_save_zero()
  {
    if (selected_index_ < 0 || !enabled_[static_cast<size_t>(selected_index_)]) {
      QMessageBox::warning(this, "Calibration", "Select and enable a motor first.");
      return;
    }
    auto & joint = joints_[static_cast<size_t>(selected_index_)];

    // Re-read rather than reusing whatever the table last showed, and
    // require a frame newer than this joint's closed-loop synchronization:
    // a stale cache -- or anything sampled before closed loop -- must never
    // become a saved zero.
    double motor_rev = 0.0;
    if (!read_post_closed_loop_motor_rev(selected_index_, motor_rev)) {
      QMessageBox::warning(
        this, "Calibration",
        "No valid post-closed-loop encoder feedback -- not saving.");
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

    // The motor stays enabled, holding its just-saved zero, so the operator
    // can move straight on to the next joint. Disable is always an
    // explicit action here.
    update_status_label();
    QMessageBox::information(
      this, "Calibration",
      QString("Saved offset for %1: %2 rad")
      .arg(QString::fromStdString(joint.name)).arg(joint.offset, 0, 'f', 6));
  }

  void on_reload_from_file()
  {
    CalibrationData reloaded;
    try {
      reloaded = load_calibration(calibration_file_);
    } catch (const std::exception & error) {
      QMessageBox::critical(
        this, "Calibration",
        QString("Failed to reload calibration file: %1").arg(error.what()));
      return;
    }

    current_limit_ = reloaded.current_limit;
    position_gain_ = reloaded.position_gain;
    velocity_gain_ = reloaded.velocity_gain;
    velocity_integrator_gain_ = reloaded.velocity_integrator_gain;
    current_limit_edit_->setText(QString::number(current_limit_, 'g', 10));
    position_gain_edit_->setText(QString::number(position_gain_, 'g', 10));
    velocity_gain_edit_->setText(QString::number(velocity_gain_, 'g', 10));
    velocity_integrator_gain_edit_->setText(QString::number(velocity_integrator_gain_, 'g', 10));

    // load_calibration() always walks kCanonicalJoints in the same fixed
    // order joints_ was originally built from, so index-for-index
    // correspondence holds; can_bus/can_id/direction/limits are canonical
    // and already re-validated by load_calibration(), only offset can have
    // actually changed on disk.
    for (size_t i = 0; i < joints_.size() && i < reloaded.joints.size(); ++i) {
      joints_[i].offset = reloaded.joints[i].offset;
    }

    // Push the fresh current_limit/gains to every already-enabled motor so
    // they take effect immediately, without disabling and re-enabling
    // (which would also reset the hold target). Offsets are not sent
    // anywhere live -- an enabled motor is always driven relative to
    // wherever it was when it was enabled (session_calibration() uses
    // offset 0 for that reason); the reloaded offset only changes what
    // on_save_zero() computes next and what the table's Saved/Offset
    // columns show.
    apply_gains_to_enabled_motors();

    update_status_label();
    QMessageBox::information(this, "Calibration", "Reloaded calibration values from file.");
  }

  void on_timer_tick()
  {
    motor_manager_->poll();
    // Only joints the operator has actually commanded get a repeated
    // Set_Input_Pos. An enabled-but-uncommanded joint is held by the motor
    // itself (Position Control + Pos Filter), and sending it a position
    // here would defeat the whole point of not commanding one at enable
    // time.
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (enabled_[i] && has_target_[i]) {
        send_target_command(static_cast<int>(i));
      }
    }
    update_table();
  }

  // Refreshes every row's State/Saved/Session/Target/Error/Vel/Motor/Offset
  // cells from live feedback. Cell widgets are created once in build_ui()
  // and only their text/colour change here -- recreating QTableWidgetItems
  // every tick (20 Hz) would make the selection and scroll position jitter.
  void update_table()
  {
    for (size_t row = 0; row < joints_.size(); ++row) {
      update_row(static_cast<int>(row));
    }
  }

  // Colour-coded axis state, driven by the motor's own Heartbeat rather
  // than by enabled_: enabled_ only records that this GUI asked for closed
  // loop, but a motor that faults drops back to idle by itself and never
  // tells us, so colouring the request instead of the reported state would
  // show green on an axis that has actually gone limp -- exactly the case
  // this table exists to catch across all 12 axes at once.
  //
  // Three angle columns are shown because calibration needs all three and
  // they only agree once the joint is calibrated:
  //   Saved   -- angle in the calibration file's frame (offset applied).
  //              Reads 0 at the saved zero, so it is what to watch when
  //              checking or re-finding a zero.
  //   Session -- angle relative to wherever the motor was when it was
  //              enabled, i.e. the frame the jog target lives in.
  //   Motor   -- the raw rotor value straight off the bus, for
  //              cross-checking against candump when something looks wrong.
  void update_row(int row)
  {
    const auto & joint = joints_[static_cast<size_t>(row)];
    auto & cells = table_items_[static_cast<size_t>(row)];

    const auto * motor = motor_manager_->motor(joint.can_id);
    const auto heartbeat = motor != nullptr &&
      motor->has_fresh_heartbeat(kHeartbeatTimeout, std::chrono::steady_clock::now()) ?
      motor->last_heartbeat() :
      std::nullopt;

    if (!heartbeat) {
      // Grey, not red: "the motor is not talking to us" and "the motor
      // says it is idle" are different problems and must not look the
      // same.
      cells[kColState]->setText("NO HEARTBEAT");
      cells[kColState]->setBackground(QBrush(kUnknownBgColor));
      cells[kColState]->setForeground(QBrush(Qt::white));
    } else {
      QString text;
      QColor background = kUnknownBgColor;
      switch (heartbeat->axis_state) {
        case gim6010_driver::AxisState::kClosedLoopControl:
          text = "CLOSED LOOP";
          background = kClosedLoopBgColor;
          break;
        case gim6010_driver::AxisState::kIdle:
          text = "IDLE";
          background = kIdleBgColor;
          break;
        default:
          // Calibration/undefined states are neither of the two the
          // operator is looking for, so they get their own colour instead
          // of being rounded to the nearer one.
          text = QString("AXIS STATE %1").arg(static_cast<uint32_t>(heartbeat->axis_state));
          break;
      }
      if (heartbeat->axis_error != 0) {
        text += QString("  fault 0x%1").arg(heartbeat->axis_error, 8, 16, QChar('0'));
      }
      cells[kColState]->setText(text);
      cells[kColState]->setBackground(QBrush(background));
      cells[kColState]->setForeground(QBrush(Qt::white));
    }

    const auto reading = fresh_motor_reading(joint);
    if (!reading) {
      for (int col = kColSaved; col <= kColMotor; ++col) {
        cells[static_cast<size_t>(col)]->setText("--");
      }
    } else {
      const double motor_rev = reading->position_rev;
      const quattro_hardware::JointCalibration saved_frame{
        joint.direction, joint.offset, kFixedGearRatio};
      const double saved_deg =
        quattro_hardware::motor_rev_to_joint_rad(motor_rev, saved_frame) * 180.0 / kPi;
      const double session_deg =
        quattro_hardware::motor_rev_to_joint_rad(motor_rev, session_calibration(joint)) *
        180.0 / kPi;
      const double target_session_deg =
        target_joint_rad_[static_cast<size_t>(row)] * 180.0 / kPi;
      // Shown in the saved frame so it lines up with the Saved column;
      // saved_rad = session_rad - offset (see on_go_to_target()).
      const double target_saved_deg = target_session_deg - joint.offset * 180.0 / kPi;
      const double session_vel_deg_s =
        quattro_hardware::motor_rev_s_to_joint_rad_s(
        reading->velocity_rev_s, session_calibration(joint)) * 180.0 / kPi;

      cells[kColSaved]->setText(QString::number(saved_deg, 'f', 3));
      cells[kColSession]->setText(QString::number(session_deg, 'f', 3));
      cells[kColTarget]->setText(QString::number(target_saved_deg, 'f', 3));
      // A tracking error only exists once a position has actually been
      // commanded. While idle the joint is free to sit anywhere, and while
      // enabled but uncommanded the "target" is just where it was when it
      // synchronized -- in neither case is the difference something the
      // motor is trying to close.
      if (!enabled_[static_cast<size_t>(row)]) {
        cells[kColError]->setText("idle");
      } else if (has_target_[static_cast<size_t>(row)]) {
        cells[kColError]->setText(QString::number(session_deg - target_session_deg, 'f', 3));
      } else {
        cells[kColError]->setText("holding");
      }
      cells[kColVel]->setText(QString::number(session_vel_deg_s, 'f', 2));
      cells[kColMotor]->setText(QString::number(motor_rev, 'f', 6));
    }

    cells[kColOffset]->setText(QString::number(joint.offset, 'f', 6));
  }

  std::string calibration_file_;
  double current_limit_{0.0};
  double position_gain_{0.0};
  double velocity_gain_{0.0};
  double velocity_integrator_gain_{0.0};
  std::vector<CalibratedJoint> joints_;
  std::vector<std::string> buses_;
  std::unique_ptr<gim6010_driver::MotorManager> motor_manager_;

  std::vector<bool> enabled_;
  // Whether the operator has requested a target for this joint since it was
  // enabled. False means "closed loop, holding itself, no Set_Input_Pos
  // sent yet" -- the state every joint is in immediately after enable.
  std::vector<bool> has_target_;
  // Gim6010Motor::encoder_sequence() at the moment this joint finished
  // synchronizing after closed-loop control was confirmed. Any encoder
  // frame at or below this is either pre-closed-loop or already consumed.
  std::vector<std::uint64_t> sync_sequence_;
  std::vector<double> target_joint_rad_;
  int selected_index_{-1};

  QTableWidget * table_{nullptr};
  std::vector<std::vector<QTableWidgetItem *>> table_items_;

  QPushButton * enable_selected_btn_{nullptr};
  QPushButton * disable_selected_btn_{nullptr};
  QPushButton * enable_all_btn_{nullptr};
  QPushButton * disable_all_btn_{nullptr};

  QLineEdit * current_limit_edit_{nullptr};
  QLineEdit * position_gain_edit_{nullptr};
  QLineEdit * velocity_gain_edit_{nullptr};
  QLineEdit * velocity_integrator_gain_edit_{nullptr};
  QPushButton * apply_gains_btn_{nullptr};
  QPushButton * save_gains_btn_{nullptr};

  QLineEdit * jog_step_deg_edit_{nullptr};
  QPushButton * minus_btn_{nullptr};
  QPushButton * plus_btn_{nullptr};
  QLineEdit * absolute_target_deg_edit_{nullptr};
  QPushButton * go_to_target_btn_{nullptr};
  QPushButton * move_to_saved_zero_btn_{nullptr};

  QPushButton * save_zero_btn_{nullptr};
  QPushButton * reload_btn_{nullptr};
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

  CalibrationData calibration;
  try {
    calibration = load_calibration(calibration_file);
  } catch (const std::exception & error) {
    QMessageBox::critical(
      nullptr, "Calibration", QString("Failed to load calibration file: %1").arg(error.what()));
    return 1;
  }

  CalibrationWindow window(calibration_file, std::move(calibration));
  window.show();
  return app.exec();
}
