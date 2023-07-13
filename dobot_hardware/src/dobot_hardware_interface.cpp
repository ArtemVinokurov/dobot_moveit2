#include <dobot_hardware/dobot_hardware_interface.hpp>
#include <dobot_bringup/commander.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>


static constexpr double PI = 3.1415926;


namespace dobot_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;


DobotHardwareInterface::~DobotHardwareInterface() {
  stop();
}

std::vector<StateInterface> DobotHardwareInterface::export_state_interfaces() {
    std::vector<StateInterface> state_interfaces;
    for (auto i = 0U; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
        state_interfaces.emplace_back(StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    }
    return state_interfaces;
}

std::vector<CommandInterface> DobotHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_.at(i)));
  }
  return command_interfaces;
}




hardware_interface::return_type DobotHardwareInterface::start() {
  //robot_->init();
  robot_->enableRobot();
  while(!robot_->isEnable());
  read();
  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(getLogger(), "Started");
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type DobotHardwareInterface::stop() {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->disableRobot();
  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(getLogger(), "Stopped");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardwareInterface::read() {
  const auto kState = robot_->real_time_data_;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    hw_positions_[i] = kState.q_actual[i] *  PI / 180.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardwareInterface::write() {
  if (std::any_of(hw_commands_.begin(), hw_commands_.end(),
                  [](double c) { return not std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  
  double temp[6];
  for (int i = 0; i < 6; i++) {
    temp[i] = hw_commands_[i] * 180 / PI;
  }
  
  robot_->servoJ(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type DobotHardwareInterface::configure(
    const hardware_interface::HardwareInfo& info) {
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %d joints. Expected %d.", info_.joints.size(), kNumberOfJoints);
    return hardware_interface::return_type::ERROR;
  }

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
  }
  std::string robot_ip;

  try {
    robot_ip = info_.hardware_parameters.at("robot_ip");
  } catch (const std::out_of_range& ex) {
    RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' not set");
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
  robot_ = std::make_unique<CR5Commander>(robot_ip);

  robot_->init();

  while(!robot_->isConnected());
  
  if (!robot_->isConnected()) {
    RCLCPP_FATAL(getLogger(), "Could not connect to robot");
    return hardware_interface::return_type::ERROR;
  }
  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  return hardware_interface::return_type::OK;

}

rclcpp::Logger DobotHardwareInterface::getLogger() {
  return rclcpp::get_logger("DobotHardwareInterface");
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dobot_hardware::DobotHardwareInterface, hardware_interface::SystemInterface)
