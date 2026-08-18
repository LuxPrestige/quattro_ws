#include <yaml-cpp/yaml.h>

#include <QApplication>
#include <QCloseEvent>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFont>
#include <QFontDatabase>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include "gim6010_driver/motor_manager.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kJogRadians = kPi / 180.0;
constexpr auto kFeedbackTimeout = std::chrono::milliseconds{2000};
constexpr auto kFeedbackRequestInterval = std::chrono::milliseconds{100};
constexpr std::uint8_t kClosedLoopControl = 8;
constexpr std::uint8_t kHeartbeatFaultMask = 0x0F;

struct JointDefinition
{
  const char * name;
  const char * label;
  int direction;
};

constexpr std::array<JointDefinition, 12> kJoints = {{
  {"front_left_hip_joint", "front_left_hip_joint (joint 0)", -1},
  {"front_left_upper_leg_joint", "front_left_upper_leg_joint (joint 1)", -1},
  {"front_left_lower_leg_joint", "front_left_lower_leg_joint (joint 2)", -1},
  {"front_right_hip_joint", "front_right_hip_joint (joint 3)", -1},
  {"front_right_upper_leg_joint", "front_right_upper_leg_joint (joint 4)", 1},
  {"front_right_lower_leg_joint", "front_right_lower_leg_joint (joint 5)", 1},
  {"back_left_hip_joint", "back_left_hip_joint (joint 6)", 1},
  {"back_left_upper_leg_joint", "back_left_upper_leg_joint (joint 7)", -1},
  {"back_left_lower_leg_joint", "back_left_lower_leg_joint (joint 8)", -1},
  {"back_right_hip_joint", "back_right_hip_joint (joint 9)", 1},
  {"back_right_upper_leg_joint", "back_right_upper_leg_joint (joint 10)", 1},
  {"back_right_lower_leg_joint", "back_right_lower_leg_joint (joint 11)", 1},
}};

void validateCalibration(const YAML::Node & root)
{
  const auto joints = root["joints"];
  if (!joints || !joints.IsMap() || joints.size() != kJoints.size()) {
    throw std::invalid_argument("The calibration YAML must contain exactly 12 joints.");
  }

  std::set<std::pair<std::string, int>> addresses;
  for (const auto & expected : kJoints) {
    const auto joint = joints[expected.name];
    if (!joint) {
      throw std::invalid_argument(std::string("Missing calibration entry: ") + expected.name);
    }
    const int direction = joint["direction"].as<int>();
    if (direction != expected.direction) {
      throw std::invalid_argument(
              std::string(expected.name) + " has a direction that differs from the reference.");
    }
    const int can_id = joint["can_id"].as<int>();
    const auto can_interface = joint["can_interface"].as<std::string>();
    if (can_id < 0 || can_id > 11) {
      throw std::invalid_argument(std::string(expected.name) + " must use a CAN ID from 0 to 11.");
    }
    const std::string expected_interface = can_id <= 5 ? "can0" : "can1";
    if (can_interface != expected_interface) {
      throw std::invalid_argument(
              std::string(expected.name) + " must use " + expected_interface + ".");
    }
    if (!addresses.emplace(can_interface, can_id).second) {
      throw std::invalid_argument("The calibration YAML contains a duplicate CAN address.");
    }
    if (!std::isfinite(joint["offset"].as<double>()) ||
      !std::isfinite(joint["kp"].as<double>()) ||
      !std::isfinite(joint["kd"].as<double>()))
    {
      throw std::invalid_argument(std::string(expected.name) +
          " contains an invalid numeric value.");
    }
  }
}

void saveCalibration(
  YAML::Node & root, const std::string & joint_name, double offset,
  const std::filesystem::path & path)
{
  root["joints"][joint_name]["offset"] = offset;
  YAML::Emitter output;
  output.SetDoublePrecision(15);
  output << root;
  if (!output.good()) {
    throw std::runtime_error("Failed to serialize the calibration YAML.");
  }

  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("Failed to open the temporary calibration file.");
    }
    stream << output.c_str() << '\n';
    stream.flush();
    if (!stream) {
      throw std::runtime_error("Failed while writing the calibration file.");
    }
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("Failed to replace the calibration file.");
  }
}

class CalibrationWindow : public QWidget
{
public:
  explicit CalibrationWindow(const std::filesystem::path & calibration_file)
  : calibration_file_(calibration_file), calibration_(YAML::LoadFile(calibration_file.string()))
  {
    validateCalibration(calibration_);
    setWindowTitle("Quattro Joint Calibration");
    setMinimumWidth(680);
    buildInterface();
    selectJoint(0);
  }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    disableAllMotors();
    event->accept();
  }

private:
  void buildInterface()
  {
    auto * main_layout = new QVBoxLayout(this);
    auto * warning = new QLabel(
      "Support the robot securely and stop controller_manager and all other CAN control programs. "
      "Each motor starts by holding its current position before any adjustment.");
    warning->setWordWrap(true);
    warning->setStyleSheet("QLabel { color: #b00020; font-weight: bold; }");
    main_layout->addWidget(warning);

    auto * joint_group = new QGroupBox("1. Select One Joint");
    auto * joint_layout = new QGridLayout(joint_group);
    for (std::size_t index = 0; index < kJoints.size(); ++index) {
      auto * button = new QPushButton(kJoints[index].label);
      button->setCheckable(true);
      button->setMinimumHeight(54);
      joint_buttons_[index] = button;
      connect(button, &QPushButton::clicked, this, [this, index]() {selectJoint(index);});
      joint_layout->addWidget(button, static_cast<int>(index / 3), static_cast<int>(index % 3));
    }
    main_layout->addWidget(joint_group);

    selected_label_ = new QLabel;
    loaded_offset_label_ = new QLabel;
    proposed_offset_label_ = new QLabel("Proposed offset: calculated after motor activation");
    position_label_ = new QLabel("Motor disabled");
    status_label_ = new QLabel("Select a joint, then enable its motor.");
    main_layout->addWidget(selected_label_);
    main_layout->addWidget(loaded_offset_label_);
    main_layout->addWidget(proposed_offset_label_);
    main_layout->addWidget(position_label_);

    auto * activation_layout = new QHBoxLayout;
    enable_button_ = new QPushButton("2. Enable Selected Motor");
    disable_button_ = new QPushButton("Disable Motor");
    activation_layout->addWidget(enable_button_);
    activation_layout->addWidget(disable_button_);
    auto * all_activation_layout = new QHBoxLayout;
    enable_all_button_ = new QPushButton("Enable All Motors");
    disable_all_button_ = new QPushButton("Disable All Motors");
    all_activation_layout->addWidget(enable_all_button_);
    all_activation_layout->addWidget(disable_all_button_);
    main_layout->addLayout(all_activation_layout);

    main_layout->addLayout(activation_layout);

    auto * jog_group = new QGroupBox("3. Adjust Joint Zero");
    auto * jog_layout = new QHBoxLayout(jog_group);
    minus_button_ = new QPushButton("-1 deg");
    plus_button_ = new QPushButton("+1 deg");
    save_button_ = new QPushButton("4. Save Current Position as Zero");
    jog_layout->addWidget(minus_button_);
    jog_layout->addWidget(plus_button_);
    jog_layout->addWidget(save_button_);
    main_layout->addWidget(jog_group);
    main_layout->addWidget(status_label_);

    connect(enable_button_, &QPushButton::clicked, this, [this]() {
        runSafely([this]() {
          enableMotor();
      });
    });
    connect(enable_all_button_, &QPushButton::clicked, this, [this]() {
        runSafely([this]() {
          enableAllMotors();
      });
    });
    connect(disable_all_button_, &QPushButton::clicked, this, [this]() {disableAllMotors();});
    connect(disable_button_, &QPushButton::clicked, this, [this]() {disableMotor();});
    connect(minus_button_, &QPushButton::clicked, this, [this]() {runSafely([this]() {jog(-1);});});
    connect(plus_button_, &QPushButton::clicked, this, [this]() {runSafely([this]() {jog(1);});});
    connect(save_button_, &QPushButton::clicked, this, [this]() {runSafely([this]() {save();});});
    updateControls();
  }

  void showProposedOffset(double offset)
  {
    proposed_offset_label_->setText(QString("Proposed offset: %1 rad (%2 deg)")
      .arg(offset, 0, 'f', 9)
      .arg(offset * 180.0 / kPi, 0, 'f', 3));
  }

  template<typename FunctionT>
  void runSafely(FunctionT function)
  {
    try {
      function();
    } catch (const std::exception & error) {
      disableAllMotors();
      QMessageBox::critical(this, "Calibration Error", error.what());
      status_label_->setText(QString("Error: %1").arg(error.what()));
    }
  }

  void selectJoint(std::size_t index)
  {
    if (motor_ != nullptr && !all_motors_active_ && index != selected_index_) {
      disableMotor();
    }
    selected_index_ = index;
    for (std::size_t button_index = 0; button_index < joint_buttons_.size(); ++button_index) {
      joint_buttons_[button_index]->setChecked(button_index == selected_index_);
    }
    const auto config = calibration_["joints"][kJoints[index].name];
    selected_label_->setText(QString("Selected: %1, %2, CAN ID %3, direction %4")
      .arg(kJoints[index].label)
      .arg(config["can_interface"].as<std::string>().c_str())
      .arg(config["can_id"].as<int>())
      .arg(config["direction"].as<int>()));
    const double loaded_offset = config["offset"].as<double>();
    loaded_offset_label_->setText(QString("Loaded offset: %1 rad (%2 deg)")
      .arg(loaded_offset, 0, 'f', 9)
      .arg(loaded_offset * 180.0 / kPi, 0, 'f', 3));
    if (all_motors_active_) {
      motor_ = all_motors_[index];
      target_ = all_targets_[index];
      kp_ = config["kp"].as<double>();
      kd_ = config["kd"].as<double>();
      direction_ = config["direction"].as<int>();
      jog_degrees_ = 0;
      showProposedOffset(direction_ * target_);
      position_label_->setText(QString("Holding motor position at %1 rad; adjustment: 0 deg")
        .arg(target_, 0, 'f', 6));
      status_label_->setText("All motors are enabled. Adjust only the selected joint.");
    } else {
      proposed_offset_label_->setText("Proposed offset: calculated after motor activation");
      status_label_->setText("The selected motor is disabled.");
    }
    updateControls();
  }

  void enableAllMotors()
  {
    if (all_motors_active_ || motor_ != nullptr) {
      return;
    }
    const auto answer = QMessageBox::warning(
      this, "Enable All Motors",
      "Enable all 12 motors and hold their current positions?\n\n"
      "The robot must be supported securely. Stop controller_manager and every other CAN sender "
      "before continuing.",
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
      return;
    }

    all_managers_[0] = std::make_unique<gim6010_driver::MotorManager>("can0");
    all_managers_[1] = std::make_unique<gim6010_driver::MotorManager>("can1");

    for (std::size_t index = 0; index < kJoints.size(); ++index) {
      const auto config = calibration_["joints"][kJoints[index].name];
      const auto can_interface = config["can_interface"].as<std::string>();
      const auto can_id = static_cast<std::uint8_t>(config["can_id"].as<int>());
      const std::size_t bus_index = can_interface == "can0" ? 0U : 1U;
      all_managers_[bus_index]->addMotor(can_id);
      all_motors_[index] = &all_managers_[bus_index]->motor(can_id);
    }

    for (std::size_t index = 0; index < kJoints.size(); ++index) {
      const auto config = calibration_["joints"][kJoints[index].name];
      const auto can_interface = config["can_interface"].as<std::string>();
      const std::size_t bus_index = can_interface == "can0" ? 0U : 1U;
      auto & motor = *all_motors_[index];
      motor.setLimits(5.0F, 10.0F);
      motor.configureMitControl();
      if (!waitForFeedback(motor, *all_managers_[bus_index])) {
        throw std::runtime_error(
                std::string("No encoder feedback was received from ") + kJoints[index].name + ".");
      }
      validatePreflight(motor, kJoints[index].name);
      all_targets_[index] = motor.encoderEstimates().position;
      motor.sendMitCommand(
        {all_targets_[index], 0.0, config["kp"].as<double>(), config["kd"].as<double>(), 0.0});
    }

    for (std::size_t index = 0; index < kJoints.size(); ++index) {
      const auto config = calibration_["joints"][kJoints[index].name];
      const auto can_interface = config["can_interface"].as<std::string>();
      const std::size_t bus_index = can_interface == "can0" ? 0U : 1U;
      auto & motor = *all_motors_[index];
      motor.enable();
      waitForClosedLoop(motor, *all_managers_[bus_index], kJoints[index].name);
      all_targets_[index] = refreshPosition(motor, *all_managers_[bus_index]);
      motor.sendMitCommand(
        {all_targets_[index], 0.0, config["kp"].as<double>(), config["kd"].as<double>(), 0.0});
    }

    all_motors_active_ = true;
    selectJoint(selected_index_);
    status_label_->setText("All 12 motors are enabled and holding their current positions.");
    updateControls();
  }

  void enableMotor()
  {
    if (motor_ != nullptr) {
      return;
    }
    const auto answer = QMessageBox::warning(
      this, "Enable One Motor",
      QString("Enable only the %1 motor?\n\n"
        "The robot must be supported securely with no load on the joint.")
      .arg(kJoints[selected_index_].label),
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
      return;
    }

    const auto config = calibration_["joints"][kJoints[selected_index_].name];
    const auto can_interface = config["can_interface"].as<std::string>();
    const auto can_id = static_cast<std::uint8_t>(config["can_id"].as<int>());
    kp_ = config["kp"].as<double>();
    kd_ = config["kd"].as<double>();
    direction_ = config["direction"].as<int>();

    manager_ = std::make_unique<gim6010_driver::MotorManager>(can_interface);
    manager_->addMotor(can_id);
    motor_ = &manager_->motor(can_id);
    motor_->setLimits(5.0F, 10.0F);
    motor_->configureMitControl();
    if (!waitForFeedback()) {
      throw std::runtime_error("No encoder feedback was received from the selected motor.");
    }
    validatePreflight(*motor_, kJoints[selected_index_].name);

    target_ = motor_->encoderEstimates().position;
    motor_->sendMitCommand({target_, 0.0, kp_, kd_, 0.0});
    motor_->enable();
    waitForClosedLoop(*motor_, *manager_, kJoints[selected_index_].name);
    target_ = refreshPosition();
    motor_->sendMitCommand({target_, 0.0, kp_, kd_, 0.0});
    jog_degrees_ = 0;
    showProposedOffset(direction_ * target_);
    position_label_->setText(QString("Holding motor position at %1 rad; adjustment: 0 deg")
      .arg(target_, 0, 'f', 6));
    status_label_->setText("The motor is enabled and holding its initial position.");
    updateControls();
  }

  bool waitForFeedback()
  {
    return waitForFeedback(*motor_, *manager_);
  }

  bool waitForFeedback(
    gim6010_driver::Gim6010Motor & motor, gim6010_driver::MotorManager & manager)
  {
    const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
    auto next_feedback_request = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_feedback_request) {
        motor.requestEncoderEstimates();
        next_feedback_request = now + kFeedbackRequestInterval;
      }
      manager.poll(std::chrono::milliseconds{20});
      if (motor.hasEncoderEstimates()) {
        return true;
      }
    }
    return false;
  }

  void validatePreflight(
    const gim6010_driver::Gim6010Motor & motor, const std::string & name) const
  {
    if (!motor.hasHeartbeat() || motor.heartbeatStale(kFeedbackTimeout)) {
      throw std::runtime_error(name + " has no fresh heartbeat.");
    }
    const auto & heartbeat = motor.heartbeat();
    if (heartbeat.axis_error != 0U ||
      (heartbeat.flags & kHeartbeatFaultMask) != 0U || heartbeat.axis_state != 1U)
    {
      throw std::runtime_error(name + " has a fault or is not idle; inspect errors before clear.");
    }
  }

  void waitForClosedLoop(
    gim6010_driver::Gim6010Motor & motor, gim6010_driver::MotorManager & manager,
    const std::string & name)
  {
    const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
      manager.poll(std::chrono::milliseconds{20});
      if (motor.hasHeartbeat() && !motor.heartbeatStale(kFeedbackTimeout)) {
        const auto & heartbeat = motor.heartbeat();
        if (heartbeat.axis_error != 0U || (heartbeat.flags & kHeartbeatFaultMask) != 0U) {
          throw std::runtime_error(name + " reported a fault while entering closed loop.");
        }
        if (heartbeat.axis_state == kClosedLoopControl) {return;}
      }
    }
    throw std::runtime_error(name + " did not enter closed-loop control.");
  }

  double refreshPosition()
  {
    if (all_motors_active_) {
      const auto config = calibration_["joints"][kJoints[selected_index_].name];
      const std::size_t bus_index =
        config["can_interface"].as<std::string>() == "can0" ? 0U : 1U;
      return refreshPosition(*motor_, *all_managers_[bus_index]);
    }
    return refreshPosition(*motor_, *manager_);
  }

  double refreshPosition(
    gim6010_driver::Gim6010Motor & motor, gim6010_driver::MotorManager & manager)
  {
    motor.requestEncoderEstimates();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{250};
    while (std::chrono::steady_clock::now() < deadline) {
      manager.poll(std::chrono::milliseconds{10});
    }
    if (!motor.hasEncoderEstimates()) {
      throw std::runtime_error("Encoder feedback is unavailable.");
    }
    return motor.encoderEstimates().position;
  }

  void jog(int joint_degrees)
  {
    if (motor_ == nullptr) {
      throw std::runtime_error("Enable the selected motor before adjusting it.");
    }
    target_ += direction_ * joint_degrees * kJogRadians;
    motor_->sendMitCommand({target_, 0.0, kp_, kd_, 0.0});
    if (all_motors_active_) {
      all_targets_[selected_index_] = target_;
    }
    jog_degrees_ += joint_degrees;
    showProposedOffset(direction_ * target_);
    position_label_->setText(QString(
        "Target motor position: %1 rad; accumulated adjustment: %2 deg")
      .arg(target_, 0, 'f', 6).arg(jog_degrees_));
  }

  void save()
  {
    if (motor_ == nullptr) {
      throw std::runtime_error("Enable the selected motor and adjust its zero before saving.");
    }
    const double motor_position = refreshPosition();
    const double offset = direction_ * motor_position;
    saveCalibration(
      calibration_, kJoints[selected_index_].name, offset, calibration_file_);
    loaded_offset_label_->setText(QString("Loaded offset: %1 rad (%2 deg)")
      .arg(offset, 0, 'f', 9)
      .arg(offset * 180.0 / kPi, 0, 'f', 3));
    showProposedOffset(offset);
    if (!all_motors_active_) {
      disableMotor();
    }
    status_label_->setText(QString("Saved the %3 offset as %1 rad (%2 deg).")
      .arg(offset, 0, 'f', 9)
      .arg(offset * 180.0 / kPi, 0, 'f', 3)
      .arg(kJoints[selected_index_].label));
  }

  void disableMotor()
  {
    if (manager_) {
      manager_->disableAll();
    }
    motor_ = nullptr;
    manager_.reset();
    position_label_->setText("Motor disabled");
    updateControls();
  }

  void disableAllMotors()
  {
    if (manager_) {
      manager_->disableAll();
    }
    for (auto & manager : all_managers_) {
      if (manager) {
        manager->disableAll();
      }
      manager.reset();
    }
    all_motors_.fill(nullptr);
    all_motors_active_ = false;
    motor_ = nullptr;
    manager_.reset();
    position_label_->setText("All motors disabled");
    updateControls();
  }

  void updateControls()
  {
    const bool active = motor_ != nullptr;
    enable_button_->setEnabled(!active && !all_motors_active_);
    disable_button_->setEnabled(active && !all_motors_active_);
    enable_all_button_->setEnabled(!active && !all_motors_active_);
    disable_all_button_->setEnabled(all_motors_active_);
    minus_button_->setEnabled(active);
    plus_button_->setEnabled(active);
    save_button_->setEnabled(active);
  }

  std::filesystem::path calibration_file_;
  YAML::Node calibration_;
  std::array<QPushButton *, 12> joint_buttons_{};
  QLabel * selected_label_{nullptr};
  QLabel * loaded_offset_label_{nullptr};
  QLabel * proposed_offset_label_{nullptr};
  QLabel * position_label_{nullptr};
  QLabel * status_label_{nullptr};
  QPushButton * enable_button_{nullptr};
  QPushButton * disable_button_{nullptr};
  QPushButton * minus_button_{nullptr};
  QPushButton * enable_all_button_{nullptr};
  QPushButton * disable_all_button_{nullptr};
  QPushButton * plus_button_{nullptr};
  QPushButton * save_button_{nullptr};
  std::size_t selected_index_{0};
  std::unique_ptr<gim6010_driver::MotorManager> manager_;
  gim6010_driver::Gim6010Motor * motor_{nullptr};
  std::array<std::unique_ptr<gim6010_driver::MotorManager>, 2> all_managers_{};
  std::array<gim6010_driver::Gim6010Motor *, 12> all_motors_{};
  std::array<double, 12> all_targets_{};
  bool all_motors_active_{false};
  double target_{0.0};
  double kp_{20.0};
  double kd_{0.5};
  int direction_{1};
  int jog_degrees_{0};
};
}  // namespace

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  QApplication::setApplicationName("quattro_calibration_gui");
  if (QFontDatabase().families().contains("Noto Sans CJK KR")) {
    QApplication::setFont(QFont("Noto Sans CJK KR", 10));
  }
  QCommandLineParser parser;
  parser.setApplicationDescription("Calibrate Quattro joints one at a time.");
  parser.addHelpOption();
  const QCommandLineOption calibration_option(
    {"c", "calibration-file"}, "Machine-specific calibration YAML", "path");
  parser.addOption(calibration_option);
  parser.process(application);
  if (!parser.isSet(calibration_option)) {
    parser.showHelp(1);
  }

  try {
    const std::filesystem::path calibration_file =
      parser.value(calibration_option).toStdString();
    if (!std::filesystem::is_regular_file(calibration_file)) {
      throw std::invalid_argument("The calibration file does not exist.");
    }
    CalibrationWindow window(calibration_file);
    window.show();
    return application.exec();
  } catch (const std::exception & error) {
    QMessageBox::critical(nullptr, "Calibration Error", error.what());
    return 1;
  }
}
