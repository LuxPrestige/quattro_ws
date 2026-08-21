// Standalone single-axis Direct Position tuning tool. It must not run at the
// same time as controller_manager or calibration_gui because all three own
// the same SocketCAN interfaces.

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleValidator>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
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
constexpr double kGearRatio = 8.0;
constexpr float kTuningVelocityLimitRevS = 5.0F;

struct Joint
{
  std::string name;
  std::string bus;
  uint8_t node_id{0};
  quattro_hardware::JointCalibration calibration;
  double lower_rad{0.0};
  double upper_rad{0.0};
};

struct TuningConfig
{
  double current_limit{5.0};
  double position_gain{20.0};
  double velocity_gain{0.16};
  double velocity_integrator_gain{0.32};
  std::vector<Joint> joints;
};

struct CanonicalJoint
{
  const char * name;
  uint8_t node_id;
  const char * bus;
  double direction;
  double lower_rad;
  double upper_rad;
};

constexpr std::array<CanonicalJoint, 12> kCanonicalJoints = {
  {{"front_left_hip_joint", 0, "can0", -1.0, -1.04, 1.04},
    {"front_left_upper_leg_joint", 1, "can0", -1.0, -1.57079632679, 2.59},
    {"front_left_lower_leg_joint", 2, "can0", -1.0, -2.9, 1.57079632679},
    {"front_right_hip_joint", 3, "can0", -1.0, -1.04, 1.04},
    {"front_right_upper_leg_joint", 4, "can0", 1.0, -1.57079632679, 2.59},
    {"front_right_lower_leg_joint", 5, "can0", 1.0, -2.9, 1.57079632679},
    {"back_left_hip_joint", 6, "can1", 1.0, -1.04, 1.04},
    {"back_left_upper_leg_joint", 7, "can1", -1.0, -1.57079632679, 2.59},
    {"back_left_lower_leg_joint", 8, "can1", -1.0, -2.9, 1.57079632679},
    {"back_right_hip_joint", 9, "can1", 1.0, -1.04, 1.04},
    {"back_right_upper_leg_joint", 10, "can1", 1.0, -1.57079632679, 2.59},
    {"back_right_lower_leg_joint", 11, "can1", 1.0, -2.9, 1.57079632679}}};

TuningConfig load_config(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node direct = root["direct_position"];
  const YAML::Node joints = root["joints"];
  if (!direct || !joints) {
    throw std::runtime_error("YAML requires top-level direct_position and joints keys");
  }

  TuningConfig config;
  config.current_limit = direct["current_limit"].as<double>();
  config.position_gain = direct["position_gain"].as<double>();
  config.velocity_gain = direct["velocity_gain"].as<double>();
  config.velocity_integrator_gain = direct["velocity_integrator_gain"].as<double>();

  for (const auto & expected : kCanonicalJoints) {
    const YAML::Node node = joints[expected.name];
    if (!node) {
      throw std::runtime_error(std::string("missing joint: ") + expected.name);
    }
    Joint joint;
    joint.name = expected.name;
    joint.bus = node["can_interface"].as<std::string>();
    joint.node_id = static_cast<uint8_t>(node["can_id"].as<int>());
    joint.calibration = {
      node["direction"].as<double>(), node["offset"].as<double>(), kGearRatio};
    joint.lower_rad = expected.lower_rad;
    joint.upper_rad = expected.upper_rad;
    if (joint.bus != expected.bus || joint.node_id != expected.node_id ||
      joint.calibration.direction != expected.direction)
    {
      throw std::runtime_error("joint mapping differs from canonical mapping: " + joint.name);
    }
    config.joints.push_back(joint);
  }
  return config;
}

void save_direct_position(const std::string & path, const TuningConfig & config)
{
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node direct = root["direct_position"];
  direct["current_limit"] = config.current_limit;
  direct["position_gain"] = config.position_gain;
  direct["velocity_gain"] = config.velocity_gain;
  direct["velocity_integrator_gain"] = config.velocity_integrator_gain;

  const std::string temporary = path + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("failed to open temporary YAML file");
    }
    output << root;
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace YAML file");
  }
}

}  // namespace

class DirectPositionTuningWindow : public QWidget
{
public:
  DirectPositionTuningWindow(std::string path, TuningConfig config)
  : path_(std::move(path)), config_(std::move(config))
  {
    std::vector<gim6010_driver::MotorRoute> routes;
    std::set<std::string> buses;
    for (const auto & joint : config_.joints) {
      buses.insert(joint.bus);
      routes.push_back({joint.node_id, joint.bus});
    }
    manager_ = std::make_unique<gim6010_driver::MotorManager>(
      std::vector<std::string>(buses.begin(), buses.end()), routes);
    build_ui();
    if (!manager_->open()) {
      QMessageBox::critical(this, "Direct Position Tuning", "Failed to open CAN interfaces");
      set_controls_enabled(false);
    }

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &DirectPositionTuningWindow::update_feedback);
    timer_->start(50);
  }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    idle_active_motor();
    manager_->close();
    QWidget::closeEvent(event);
  }

private:
  void build_ui()
  {
    setWindowTitle("Quattro Direct Position Tuning");
    auto * layout = new QVBoxLayout(this);
    auto * form = new QFormLayout();

    joint_box_ = new QComboBox(this);
    for (const auto & joint : config_.joints) {
      joint_box_->addItem(QString::fromStdString(joint.name));
    }
    connect(
      joint_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
      [this](int) {idle_active_motor(); update_feedback();});

    current_limit_ = make_number(QString::number(config_.current_limit, 'g', 10), 0.001, 100.0);
    position_gain_ = make_number(QString::number(config_.position_gain, 'g', 10), 0.0, 100000.0);
    velocity_gain_ = make_number(QString::number(config_.velocity_gain, 'g', 10), 0.0, 100000.0);
    velocity_integrator_gain_ = make_number(
      QString::number(config_.velocity_integrator_gain, 'g', 10), 0.0, 100000.0);
    relative_target_deg_ = make_number("1.0", -10.0, 10.0);

    form->addRow("Motor", joint_box_);
    form->addRow("Current limit (A)", current_limit_);
    form->addRow("Position gain", position_gain_);
    form->addRow("Velocity gain", velocity_gain_);
    form->addRow("Velocity integrator gain", velocity_integrator_gain_);
    form->addRow("Relative target (deg)", relative_target_deg_);
    layout->addLayout(form);

    auto * actions = new QHBoxLayout();
    enable_ = new QPushButton("Apply and Enable", this);
    send_target_ = new QPushButton("Send Relative Target", this);
    disable_ = new QPushButton("Idle Motor", this);
    actions->addWidget(enable_);
    actions->addWidget(send_target_);
    actions->addWidget(disable_);
    layout->addLayout(actions);

    save_yaml_ = new QPushButton("Save Values to YAML", this);
    layout->addWidget(save_yaml_);
    measurement_ = new QLabel("No feedback", this);
    status_ = new QLabel("Only one motor can be active.", this);
    status_->setWordWrap(true);
    layout->addWidget(measurement_);
    layout->addWidget(status_);

    connect(enable_, &QPushButton::clicked, this, &DirectPositionTuningWindow::enable_motor);
    connect(send_target_, &QPushButton::clicked, this, &DirectPositionTuningWindow::send_target);
    connect(disable_, &QPushButton::clicked, this, &DirectPositionTuningWindow::idle_active_motor);
    connect(save_yaml_, &QPushButton::clicked, this, &DirectPositionTuningWindow::save_yaml);
    send_target_->setEnabled(false);
  }

  QLineEdit * make_number(const QString & value, double minimum, double maximum)
  {
    auto * edit = new QLineEdit(value, this);
    edit->setValidator(new QDoubleValidator(minimum, maximum, 9, edit));
    return edit;
  }

  bool read_inputs(TuningConfig & output)
  {
    bool ok_current = false;
    bool ok_position = false;
    bool ok_velocity = false;
    bool ok_integrator = false;
    output = config_;
    output.current_limit = current_limit_->text().toDouble(&ok_current);
    output.position_gain = position_gain_->text().toDouble(&ok_position);
    output.velocity_gain = velocity_gain_->text().toDouble(&ok_velocity);
    output.velocity_integrator_gain =
      velocity_integrator_gain_->text().toDouble(&ok_integrator);
    if (!ok_current || !ok_position || !ok_velocity || !ok_integrator ||
      !std::isfinite(output.current_limit) || !std::isfinite(output.position_gain) ||
      !std::isfinite(output.velocity_gain) || !std::isfinite(output.velocity_integrator_gain) ||
      !(output.current_limit > 0.0) || output.position_gain < 0.0 ||
      output.velocity_gain < 0.0 || output.velocity_integrator_gain < 0.0)
    {
      QMessageBox::warning(this, "Direct Position Tuning", "Enter valid finite tuning values.");
      return false;
    }
    return true;
  }

  bool read_fresh_position(const Joint & joint, double & joint_rad)
  {
    // Get_Error (0x03) goes unanswered on this firmware even though
    // Get_Encoder_Estimates (0x09) and Heartbeat (0x01) both arrive on
    // their own (confirmed on the bus with candump against node 0-2).
    // Heartbeat already carries axis_error, so use that for the fault check
    // instead of blocking forever on a response that never arrives.
    // Nothing is requested here: 0x09 is broadcast by the motors, so this
    // only waits for one to land.
    for (int attempt = 0; attempt < 20; ++attempt) {
      QThread::msleep(10);
      manager_->poll();
      const auto * motor = manager_->motor(joint.node_id);
      if (motor && motor->last_encoder_estimate() && motor->last_heartbeat()) {
        const auto heartbeat = *motor->last_heartbeat();
        if (heartbeat.axis_error != 0) {
          status_->setText(
            QString("Motor fault: axis_error=0x%1").arg(heartbeat.axis_error, 8, 16, QChar('0')));
          return false;
        }
        joint_rad = quattro_hardware::motor_rev_to_joint_rad(
          motor->last_encoder_estimate()->position_rev, joint.calibration);
        return true;
      }
    }
    status_->setText("No fresh encoder/heartbeat response; refusing to enable.");
    return false;
  }

  void enable_motor()
  {
    idle_active_motor();
    TuningConfig entered;
    if (!read_inputs(entered)) {
      return;
    }
    config_ = entered;
    const int index = joint_box_->currentIndex();
    if (index < 0) {
      return;
    }
    const auto & joint = config_.joints[static_cast<size_t>(index)];
    double current_rad = 0.0;
    if (!read_fresh_position(joint, current_rad)) {
      return;
    }

    const bool configured =
      manager_->send_set_limits(
      joint.node_id, kTuningVelocityLimitRevS, static_cast<float>(config_.current_limit)) &&
      manager_->send_set_pos_gain(joint.node_id, static_cast<float>(config_.position_gain)) &&
      manager_->send_set_vel_gains(
      joint.node_id, static_cast<float>(config_.velocity_gain),
      static_cast<float>(config_.velocity_integrator_gain)) &&
      manager_->send_set_controller_mode(
      joint.node_id, gim6010_driver::ControlMode::kPositionControl,
      gim6010_driver::InputMode::kDirect);
    if (!configured) {
      status_->setText("Failed to configure the selected motor.");
      return;
    }

    // Write the hold-at-current-position target before requesting
    // closed-loop control, not after: Input_Pos keeps whatever it was last
    // set to (possibly from a previous session, far away) until we write
    // it, regardless of axis state. Setting it while still idle means
    // closed-loop starts from a stationary target instead of chasing a
    // stale one for the brief window until this command lands (on real
    // hardware this window was long enough to visibly snap the joint in
    // the wrong direction, sometimes 180+ degrees, the instant Apply and
    // Enable was pressed -- same fix as QuattroSystem::activate_joint(),
    // docs/packages/quattro_hardware.md section 2).
    gim6010_driver::SetInputPosCommand hold_command;
    hold_command.position_rev = static_cast<float>(
      quattro_hardware::joint_rad_to_motor_rev(current_rad, joint.calibration));
    if (!manager_->send_set_input_pos(joint.node_id, hold_command)) {
      status_->setText("Failed to send the initial hold position.");
      return;
    }

    if (!manager_->send_set_axis_state(
        joint.node_id, gim6010_driver::AxisState::kClosedLoopControl))
    {
      status_->setText("Failed to enable the selected motor.");
      manager_->send_set_axis_state(joint.node_id, gim6010_driver::AxisState::kIdle);
      return;
    }

    active_index_ = index;
    target_rad_ = current_rad;
    send_target_->setEnabled(true);
    joint_box_->setEnabled(false);
    status_->setText("Enabled at the measured position. Enter a relative target and send it.");
  }

  void send_target()
  {
    if (active_index_ < 0) {
      return;
    }
    bool valid = false;
    const double degrees = relative_target_deg_->text().toDouble(&valid);
    if (!valid || !std::isfinite(degrees)) {
      QMessageBox::warning(this, "Direct Position Tuning", "Enter a valid relative target.");
      return;
    }
    const auto & joint = config_.joints[static_cast<size_t>(active_index_)];
    const double requested = target_rad_ + degrees * kPi / 180.0;
    if (requested < joint.lower_rad || requested > joint.upper_rad) {
      QMessageBox::warning(
        this, "Direct Position Tuning", "Requested target is outside the URDF joint limit.");
      return;
    }
    target_rad_ = requested;
    send_position(joint, target_rad_);
  }

  void send_position(const Joint & joint, double target)
  {
    gim6010_driver::SetInputPosCommand command;
    command.position_rev = static_cast<float>(
      quattro_hardware::joint_rad_to_motor_rev(target, joint.calibration));
    if (!manager_->send_set_input_pos(joint.node_id, command)) {
      idle_active_motor();
      status_->setText("Position command rejected; motor changed to Idle.");
    }
  }

  void idle_active_motor()
  {
    if (active_index_ >= 0) {
      const auto & joint = config_.joints[static_cast<size_t>(active_index_)];
      manager_->send_set_axis_state(joint.node_id, gim6010_driver::AxisState::kIdle);
    }
    active_index_ = -1;
    send_target_->setEnabled(false);
    joint_box_->setEnabled(true);
    status_->setText("Motor is Idle.");
  }

  void update_feedback()
  {
    if (!manager_ || config_.joints.empty()) {
      return;
    }
    manager_->poll();
    const int index = active_index_ >= 0 ? active_index_ : joint_box_->currentIndex();
    if (index < 0) {
      return;
    }
    const auto & joint = config_.joints[static_cast<size_t>(index)];
    const auto * motor = manager_->motor(joint.node_id);
    if (!motor || !motor->last_encoder_estimate()) {
      measurement_->setText("No feedback");
      return;
    }
    if (active_index_ >= 0) {
      const auto now = std::chrono::steady_clock::now();
      const auto heartbeat = motor->last_heartbeat();
      if (!motor->has_fresh_heartbeat(std::chrono::milliseconds(400), now) ||
        (heartbeat && heartbeat->axis_error != 0))
      {
        idle_active_motor();
        status_->setText("Heartbeat became stale or reported a fault; motor changed to Idle.");
        return;
      }
      send_position(joint, target_rad_);
    }
    const auto estimate = *motor->last_encoder_estimate();
    const double position = quattro_hardware::motor_rev_to_joint_rad(
      estimate.position_rev, joint.calibration);
    const double velocity = quattro_hardware::motor_rev_s_to_joint_rad_s(
      estimate.velocity_rev_s, joint.calibration);
    const double error = active_index_ >= 0 ? target_rad_ - position : 0.0;
    measurement_->setText(
      QString("position=%1 deg | target=%2 deg | error=%3 deg | velocity=%4 deg/s")
      .arg(position * 180.0 / kPi, 0, 'f', 3)
      .arg(active_index_ >= 0 ? target_rad_ * 180.0 / kPi : position * 180.0 / kPi, 0, 'f', 3)
      .arg(error * 180.0 / kPi, 0, 'f', 3)
      .arg(velocity * 180.0 / kPi, 0, 'f', 3));
  }

  void save_yaml()
  {
    TuningConfig entered;
    if (!read_inputs(entered)) {
      return;
    }
    try {
      save_direct_position(path_, entered);
      config_.current_limit = entered.current_limit;
      config_.position_gain = entered.position_gain;
      config_.velocity_gain = entered.velocity_gain;
      config_.velocity_integrator_gain = entered.velocity_integrator_gain;
      QMessageBox::information(this, "Direct Position Tuning", "Saved tuning values to YAML.");
    } catch (const std::exception & error) {
      QMessageBox::critical(this, "Direct Position Tuning", error.what());
    }
  }

  void set_controls_enabled(bool enabled)
  {
    enable_->setEnabled(enabled);
    send_target_->setEnabled(false);
    disable_->setEnabled(enabled);
  }

  std::string path_;
  TuningConfig config_;
  std::unique_ptr<gim6010_driver::MotorManager> manager_;
  int active_index_{-1};
  double target_rad_{0.0};

  QComboBox * joint_box_{nullptr};
  QLineEdit * current_limit_{nullptr};
  QLineEdit * position_gain_{nullptr};
  QLineEdit * velocity_gain_{nullptr};
  QLineEdit * velocity_integrator_gain_{nullptr};
  QLineEdit * relative_target_deg_{nullptr};
  QPushButton * enable_{nullptr};
  QPushButton * send_target_{nullptr};
  QPushButton * disable_{nullptr};
  QPushButton * save_yaml_{nullptr};
  QLabel * measurement_{nullptr};
  QLabel * status_{nullptr};
  QTimer * timer_{nullptr};
};

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  std::string path;
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) == "--calibration-file" && index + 1 < argc) {
      path = argv[++index];
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "Usage: direct_position_tuning_gui --calibration-file <path>\n");
    return 1;
  }

  try {
    DirectPositionTuningWindow window(path, load_config(path));
    window.show();
    return application.exec();
  } catch (const std::exception & error) {
    QMessageBox::critical(nullptr, "Direct Position Tuning", error.what());
    return 1;
  }
}
