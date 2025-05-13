#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "rsi/kukarsiinterface.h"

namespace kuka_rsi_control
{

class KukaInterfacePlugin : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaInterfacePlugin)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::shared_ptr<rsi::KukaRsiInterface> rsi_interface_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
  size_t joint_count_;
};

}  // namespace kuka_rsi_control
