#include "kuka_interface_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <limits>

namespace kuka_rsi_control
{

hardware_interface::CallbackReturn KukaInterfacePlugin::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  joint_count_ = info.joints.size();
  joint_positions_.resize(joint_count_, 0.0);
  joint_commands_.resize(joint_count_, 0.0);

  // Read RSI config from parameters
  std::string rsi_host = info.hardware_parameters.at("rsi_host");
  uint16_t rsi_port = static_cast<uint16_t>(std::stoi(info.hardware_parameters.at("rsi_port")));

  // Create the RSI interface
  rsi_interface_ = std::make_shared<rsi::KukaRsiInterface>(
      rsi_host,
      rsi_port,
      joint_count_,
      [this](const Eigen::ArrayXd &pos) {
        for (size_t i = 0; i < joint_count_; ++i)
          joint_positions_[i] = pos[i];
      });

  if (!rsi_interface_->start())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaInterfacePlugin"), "Failed to start RSI interface.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaInterfacePlugin::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_count_; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaInterfacePlugin::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_count_; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type KukaInterfacePlugin::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Joint positions are updated via callback inside KukaRsiInterface
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaInterfacePlugin::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  Eigen::ArrayXd cmd(joint_count_);
  for (size_t i = 0; i < joint_count_; ++i)
    cmd[i] = joint_commands_[i];

  rsi_interface_->setJointPositionsRad(cmd);
  return hardware_interface::return_type::OK;
}

}  // namespace kuka_rsi_control

PLUGINLIB_EXPORT_CLASS(kuka_rsi_control::KukaInterfacePlugin, hardware_interface::SystemInterface)
