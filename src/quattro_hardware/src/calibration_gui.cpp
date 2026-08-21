// Standalone Qt calibration GUI. Deliberately independent of ros2_control/
// controller_manager (docs/packages/quattro_hardware.md section 5) -- it
// talks to gim6010_driver directly and must not run at the same time as
// hardware.launch.py (docs/calibration.md safety condition).
//
// Uses the same Direct Position path as runtime bringup. The calibration
// file's common current limit and position/velocity gains are applied before
// an axis enters closed-loop control.

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QFontDatabase>
#include <QFontMetrics>
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
#include <chrono>
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
constexpr double kJogStepRad = kPi / 180.0;  // 1 degree
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

// Explicit foreground *and* background on the state chip: the Qt palette
// this runs under is not known ahead of time, and a bare coloured
// foreground can end up unreadable on a dark theme. These are the only
// colours in the GUI, so they are spelled out here rather than in a
// stylesheet file.
constexpr const char * kClosedLoopChipStyle =
  "background-color: #1b7f3b; color: #ffffff; padding: 4px; font-weight: bold;";
constexpr const char * kIdleChipStyle =
  "background-color: #b3261e; color: #ffffff; padding: 4px; font-weight: bold;";
constexpr const char * kUnknownChipStyle =
  "background-color: #6b6b6b; color: #ffffff; padding: 4px; font-weight: bold;";

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
};

struct CalibrationData
{
  double current_limit{0.0};
  double position_gain{0.0};
  double velocity_gain{0.0};
  double velocity_integrator_gain{0.0};
  std::vector<CalibratedJoint> joints;
};

CalibrationData load_calibration(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node direct_position = root["direct_position"];
  if (!direct_position) {
    throw std::runtime_error("calibration file has no top-level 'direct_position' key: " + path);
  }
  const YAML::Node joints_node = root["joints"];
  if (!joints_node) {
    throw std::runtime_error("calibration file has no top-level 'joints' key: " + path);
  }

  CalibrationData result;
  result.current_limit = direct_position["current_limit"].as<double>();
  result.position_gain = direct_position["position_gain"].as<double>();
  result.velocity_gain = direct_position["velocity_gain"].as<double>();
  result.velocity_integrator_gain = direct_position["velocity_integrator_gain"].as<double>();
  if (!(result.current_limit > 0.0) || result.position_gain < 0.0 ||
    result.velocity_gain < 0.0 || result.velocity_integrator_gain < 0.0)
  {
    throw std::runtime_error(
      "Direct Position current_limit must be positive and gains must be non-negative");
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
    result.joints.push_back(joint);
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
  CalibrationWindow(std::string calibration_file, CalibrationData calibration)
  : calibration_file_(std::move(calibration_file)),
    current_limit_(calibration.current_limit),
    position_gain_(calibration.position_gain),
    velocity_gain_(calibration.velocity_gain),
    velocity_integrator_gain_(calibration.velocity_integrator_gain),
    joints_(std::move(calibration.joints))
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
    update_state_label();
    update_measured_label();
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
    move_to_saved_zero_btn_ = new QPushButton("Move to Saved Zero", this);
    save_btn_ = new QPushButton("Save Current Position as Zero", this);
    reload_btn_ = new QPushButton("Reload Calibration from File", this);
    status_label_ = new QLabel(this);
    status_label_->setWordWrap(true);
    measured_label_ = new QLabel(this);
    // Fixed-width digits: this label is refreshed every tick, and a
    // proportional font makes the numbers jitter sideways as they change,
    // which is exactly the wrong behaviour for a readout you watch while
    // jogging a joint by hand.
    measured_label_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    measured_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    // The readout is always exactly four lines (the no-feedback text is
    // padded to match), so reserve them up front. Without this the label
    // only gets the height it happened to need when the window was first
    // laid out, and the last line -- the raw motor reading -- is silently
    // clipped once feedback starts arriving.
    measured_label_->setMinimumHeight(QFontMetrics(measured_label_->font()).lineSpacing() * 4);
    state_label_ = new QLabel(this);
    state_label_->setAlignment(Qt::AlignCenter);

    connect(enable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_selected);
    connect(
      disable_selected_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_selected);
    connect(enable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_enable_all);
    connect(disable_all_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_disable_all);
    connect(minus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_minus);
    connect(plus_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_jog_plus);
    connect(
      move_to_saved_zero_btn_, &QPushButton::clicked, this,
      &CalibrationWindow::on_move_to_saved_zero);
    connect(save_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_save_zero);
    connect(reload_btn_, &QPushButton::clicked, this, &CalibrationWindow::on_reload_from_file);

    auto * layout = new QVBoxLayout(this);
    layout->addWidget(
      new QLabel(QString("Calibration file: %1").arg(QString::fromStdString(calibration_file_))));
    layout->addWidget(reload_btn_);
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

    layout->addWidget(move_to_saved_zero_btn_);

    auto * jog_layout = new QHBoxLayout();
    jog_layout->addWidget(minus_btn_);
    jog_layout->addWidget(plus_btn_);
    layout->addLayout(jog_layout);

    layout->addWidget(save_btn_);

    auto * live_group = new QGroupBox("Live feedback", this);
    auto * live_layout = new QVBoxLayout(live_group);
    live_layout->addWidget(state_label_);
    live_layout->addWidget(measured_label_);
    layout->addWidget(live_group);

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

  // Blocks until a reading newer than kFeedbackTimeout has arrived for this
  // joint, or ~200ms elapses. Nothing is requested: the motors broadcast
  // 0x09 by themselves, so the only thing worth checking is freshness.
  // Freshness is what makes this safe to call before enabling -- a stale
  // cached estimate left over from earlier in the session would otherwise
  // become the hold target and jerk the joint back to where it used to be.
  bool read_fresh_motor_rev(int index, double & out_motor_rev)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    for (int attempt = 0; attempt < 20; ++attempt) {
      motor_manager_->poll();
      if (const auto reading = fresh_motor_reading(joint)) {
        out_motor_rev = reading->position_rev;
        return true;
      }
      QThread::msleep(10);
    }
    return false;
  }

  // The non-blocking half of the same idea, used by the live display.
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

  void send_hold_command(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    const auto calibration = session_calibration(joint);
    gim6010_driver::SetInputPosCommand command;
    command.position_rev = static_cast<float>(quattro_hardware::joint_rad_to_motor_rev(
      target_joint_rad_[static_cast<size_t>(index)], calibration));
    if (!motor_manager_->send_set_input_pos(joint.can_id, command)) {
      status_label_->setText(
        QString("WARNING: %1 Direct Position hold command rejected")
        .arg(QString::fromStdString(joint.name)));
    }
  }

  void set_motor_closed_loop(int index)
  {
    const auto & joint = joints_[static_cast<size_t>(index)];
    motor_manager_->send_set_limits(
      joint.can_id, kCalibrationVelocityLimitRevS, static_cast<float>(current_limit_));
    motor_manager_->send_set_pos_gain(joint.can_id, static_cast<float>(position_gain_));
    motor_manager_->send_set_vel_gains(
      joint.can_id, static_cast<float>(velocity_gain_),
      static_cast<float>(velocity_integrator_gain_));

    double motor_rev = 0.0;
    if (!read_fresh_motor_rev(index, motor_rev)) {
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
      gim6010_driver::InputMode::kDirect);
    // Write the hold-at-current-position target before requesting
    // closed-loop control, not after: Input_Pos keeps whatever it was last
    // set to (possibly from a previous session, far away) until we write
    // it, regardless of axis state. Setting it while still idle removes the
    // window where closed-loop would otherwise chase a stale target the
    // instant it engages (same fix as QuattroSystem::activate_joint(),
    // docs/packages/quattro_hardware.md section 2).
    send_hold_command(index);
    motor_manager_->send_set_axis_state(joint.can_id, gim6010_driver::AxisState::kClosedLoopControl);
    enabled_[static_cast<size_t>(index)] = true;
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
    // Switching the dropdown only changes which joint the jog buttons and
    // Enable/Disable Selected act on -- it does not touch any other
    // motor's enabled state. Selecting a different joint used to idle
    // whichever single motor was previously active; that made it
    // impossible to hold several joints enabled at once while calibrating
    // them one at a time (e.g. holding one leg fixed as a reference while
    // adjusting an adjacent joint). Each joint's enabled state is now
    // independent of selection.
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
    // Deliberately does not idle any other already-enabled motor first (see
    // on_selection_changed()) -- enabling the selected joint only ever adds
    // to the set of currently-held motors.
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
    target_joint_rad_[static_cast<size_t>(selected_index_)] = joint.offset;
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
    if (!read_fresh_motor_rev(selected_index_, motor_rev)) {
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
    // load_calibration() always walks kCanonicalJoints in the same fixed
    // order joints_ was originally built from, so index-for-index
    // correspondence holds; can_bus/can_id/direction are canonical and
    // already re-validated by load_calibration(), only offset can have
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
    // on_save_zero() computes next and what the status label shows below.
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

    update_status_label();
    QMessageBox::information(this, "Calibration", "Reloaded calibration values from file.");
  }

  void on_timer_tick()
  {
    motor_manager_->poll();
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (enabled_[i]) {
        send_hold_command(static_cast<int>(i));
      }
    }
    update_state_label();
    update_measured_label();
  }

  // Colour-coded axis state of the selected joint, driven by the motor's own
  // Heartbeat rather than by enabled_. enabled_ only records that we asked
  // for closed loop; a motor that faults drops back to idle by itself and
  // never tells us, so colouring the request instead of the reported state
  // would show green on a joint that has actually gone limp -- exactly the
  // case this indicator exists to catch.
  void update_state_label()
  {
    if (selected_index_ < 0) {
      state_label_->setStyleSheet(kUnknownChipStyle);
      state_label_->setText("NO JOINT SELECTED");
      return;
    }
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    const auto * motor = motor_manager_->motor(joint.can_id);
    const auto heartbeat = motor != nullptr &&
      motor->has_fresh_heartbeat(kHeartbeatTimeout, std::chrono::steady_clock::now()) ?
      motor->last_heartbeat() :
      std::nullopt;
    if (!heartbeat) {
      // Grey, not red: "the motor is not talking to us" and "the motor says
      // it is idle" are different problems and must not look the same.
      state_label_->setStyleSheet(kUnknownChipStyle);
      state_label_->setText("NO HEARTBEAT");
      return;
    }

    QString text;
    switch (heartbeat->axis_state) {
      case gim6010_driver::AxisState::kClosedLoopControl:
        state_label_->setStyleSheet(kClosedLoopChipStyle);
        text = "CLOSED LOOP";
        break;
      case gim6010_driver::AxisState::kIdle:
        state_label_->setStyleSheet(kIdleChipStyle);
        text = "IDLE";
        break;
      default:
        // Calibration/undefined states are neither of the two the operator
        // is looking for, so they get their own colour instead of being
        // rounded to the nearer one.
        state_label_->setStyleSheet(kUnknownChipStyle);
        text = QString("AXIS STATE %1")
          .arg(static_cast<uint32_t>(heartbeat->axis_state));
        break;
    }
    if (heartbeat->axis_error != 0) {
      text += QString("   fault 0x%1").arg(heartbeat->axis_error, 8, 16, QChar('0'));
    }
    state_label_->setText(text);
  }

  // Live readout of the selected joint, refreshed every timer tick off the
  // motors' own 0x09 broadcast. Three frames are shown because calibration
  // needs all three and they only agree once the joint is calibrated:
  //   saved  -- angle in the calibration file's frame (offset applied).
  //             This is the number that reads 0 at the saved zero, so it is
  //             what you watch when checking or re-finding a zero.
  //   session -- angle relative to wherever the motor was when it was
  //             enabled, i.e. the same frame the jog target lives in.
  //   motor  -- the raw rotor value straight off the bus, for cross-checking
  //             against candump when something looks wrong.
  void update_measured_label()
  {
    if (selected_index_ < 0) {
      measured_label_->setText("No joint selected.");
      return;
    }
    const auto & joint = joints_[static_cast<size_t>(selected_index_)];
    const auto reading = fresh_motor_reading(joint);
    if (!reading) {
      // Padded to the same four lines as the normal readout: this label is
      // rewritten every tick, and a changing line count makes the whole
      // window resize under the operator's cursor as feedback comes and
      // goes.
      measured_label_->setText(
        QString("%1  (node %2 on %3)\n"
        "no feedback -- nothing newer than %4 ms has arrived\n"
        "this motor is either not broadcasting Get_Encoder_Estimates (0x09)\n"
        "or has dropped off the bus")
        .arg(QString::fromStdString(joint.name))
        .arg(joint.can_id)
        .arg(QString::fromStdString(joint.can_bus))
        .arg(kFeedbackTimeout.count()));
      return;
    }

    const double motor_rev = reading->position_rev;
    const quattro_hardware::JointCalibration saved_frame{
      joint.direction, joint.offset, kFixedGearRatio};
    const double saved_deg =
      quattro_hardware::motor_rev_to_joint_rad(motor_rev, saved_frame) * 180.0 / kPi;
    const double session_deg =
      quattro_hardware::motor_rev_to_joint_rad(motor_rev, session_calibration(joint)) * 180.0 / kPi;
    const double target_deg =
      target_joint_rad_[static_cast<size_t>(selected_index_)] * 180.0 / kPi;
    const double session_vel_deg_s =
      quattro_hardware::motor_rev_s_to_joint_rad_s(
      reading->velocity_rev_s, session_calibration(joint)) * 180.0 / kPi;

    measured_label_->setText(
      QString("%1  (node %2 on %3)\n"
      "saved   %4 deg\n"
      "session %5 deg   target %6 deg   error %7\n"
      "motor   %8 rev   %9 deg/s")
      .arg(QString::fromStdString(joint.name))
      .arg(joint.can_id)
      .arg(QString::fromStdString(joint.can_bus))
      .arg(saved_deg, 9, 'f', 3)
      .arg(session_deg, 9, 'f', 3)
      .arg(target_deg, 9, 'f', 3)
      // Only meaningful while the motor is actually holding: when it is
      // idle the target is whatever was last commanded and the joint is
      // free to sit anywhere, so the difference is not a tracking error.
      .arg(
        enabled_[static_cast<size_t>(selected_index_)] ?
        QString("%1 deg").arg(session_deg - target_deg, 9, 'f', 3) :
        QString("idle").rightJustified(13))
      .arg(motor_rev, 10, 'f', 6)
      .arg(session_vel_deg_s, 8, 'f', 2));
  }

  enum class Mode { kNone, kSingle, kAll };

  std::string calibration_file_;
  double current_limit_{0.0};
  double position_gain_{0.0};
  double velocity_gain_{0.0};
  double velocity_integrator_gain_{0.0};
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
  QPushButton * move_to_saved_zero_btn_{nullptr};
  QPushButton * save_btn_{nullptr};
  QPushButton * reload_btn_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * measured_label_{nullptr};
  QLabel * state_label_{nullptr};
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
