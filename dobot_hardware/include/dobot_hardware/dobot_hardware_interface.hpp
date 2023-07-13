#ifndef DOBOT_HARDWARE_INTERFACE__DOBOT_HARDWARE_INTERFACE_HPP_
#define DOBOT_HARDWARE_INTERFACE__DOBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

// #include <hardware_interface/visibility_control.h>
#include "hardware_interface/visibility_control.h"
#include <dobot_bringup/commander.hpp>
#include "hardware_interface/handle.hpp"
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <hardware_interface/visibility_control.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/state.hpp"
#include <chrono>


namespace dobot_hardware {
class DobotHardwareInterface: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
    public:
   //   hardware_interface::return_type prepare_command_mode_switch(
   //      const std::vector<std::string>& start_interfaces,
   //      const std::vector<std::string>& stop_interfaces) override;
   //   hardware_interface::return_type perform_command_mode_switch(
   //      const std::vector<std::string>& start_interfaces,
   //      const std::vector<std::string>& stop_interfaces) override;
     RCLCPP_SHARED_PTR_DEFINITIONS(DobotHardwareInterface)

     DobotHardwareInterface() = default;
     ~DobotHardwareInterface();

     hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;
     std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
     std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


     hardware_interface::return_type start() override;
     hardware_interface::return_type stop() override;
     hardware_interface::return_type read() override;
     hardware_interface::return_type write() override;
     
     static const size_t kNumberOfJoints = 6;

    private:
     std::unique_ptr<CR5Commander> robot_;
     std::array<double, kNumberOfJoints> hw_commands_{0, 0, 0, 0, 0, 0};
     std::array<double, kNumberOfJoints> hw_positions_{0, 0, 0, 0, 0, 0};
     std::array<double, kNumberOfJoints> hw_velocities_{0, 0, 0, 0, 0, 0};
     static rclcpp::Logger getLogger();


};
}

#endif