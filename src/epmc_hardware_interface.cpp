// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "epmc_hardware_interface/epmc_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

namespace epmc_hardware_interface
{
  hardware_interface::CallbackReturn EPMCHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.motorA_wheel_name = info_.hardware_parameters["motorA_wheel_name"];
    cfg_.motorB_wheel_name = info_.hardware_parameters["motorB_wheel_name"];
    cfg_.port = info_.hardware_parameters["port"];
    cfg_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];

    motorA_.setup(cfg_.motorA_wheel_name);
    motorB_.setup(cfg_.motorB_wheel_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMCHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMCHardwareInterface"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMCHardwareInterface"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMCHardwareInterface"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMCHardwareInterface"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EPMCHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorA_.name, hardware_interface::HW_IF_POSITION, &motorA_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorA_.name, hardware_interface::HW_IF_VELOCITY, &motorA_.angVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorB_.name, hardware_interface::HW_IF_POSITION, &motorB_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorB_.name, hardware_interface::HW_IF_VELOCITY, &motorB_.angVel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMCHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        motorA_.name, hardware_interface::HW_IF_VELOCITY, &motorA_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        motorB_.name, hardware_interface::HW_IF_VELOCITY, &motorB_.cmdAngVel));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMCHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Configuring ...please wait...");
    if (epmc_.connected())
    {
      epmc_.disconnect();
    }
    epmc_.connect(cfg_.port, 115200, 100);
    for (int i = 1; i <= 6; i += 1)
    { // wait for the smc to fully setup
      delay_ms(1000);
      RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "configuring controller: %d sec", (i));
    }
    epmc_.sendTargetVel(0.000, 0.000); // targetA, targetB

    int cmd_timeout = std::stoi(cfg_.cmd_vel_timeout_ms.c_str());
    epmc_.setCmdTimeout(cmd_timeout); // set motor command timeout
    epmc_.getCmdTimeout(cmd_timeout);
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));

    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMCHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Cleaning up ...please wait...");
    if (epmc_.connected())
    {
      epmc_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMCHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Activating ...please wait...");
    if (!epmc_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    epmc_.sendTargetVel(0.000, 0.000); // targetA, targetB
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMCHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Deactivating ...please wait...");
    epmc_.sendTargetVel(0.000, 0.000); // targetA, targetB
    RCLCPP_INFO(rclcpp::get_logger("EPMCHardwareInterface"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EPMCHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!epmc_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    try
    {

      float motorA_angPos, motorB_angPos;
      float motorA_angVel, motorB_angVel;

      epmc_.getMotorsPos(motorA_angPos, motorB_angPos); // gets angPosA, angPosB
      epmc_.getMotorsVel(motorA_angVel, motorB_angVel); // gets angVelA, angVelB

      motorA_.angPos = (double)motorA_angPos;
      motorB_.angPos = (double)motorB_angPos;

      motorA_.angVel = (double)motorA_angVel;
      motorB_.angVel = (double)motorB_angVel;
    }
    catch (...)
    {
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type epmc_hardware_interface ::EPMCHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!epmc_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    float motorA_cmdAngVel, motorB_cmdAngVel;

    motorA_cmdAngVel = (float)motorA_.cmdAngVel;
    motorB_cmdAngVel = (float)motorB_.cmdAngVel;

    epmc_.sendTargetVel(motorA_cmdAngVel, motorB_cmdAngVel); // targetA, targetB

    return hardware_interface::return_type::OK;
  }

} // namespace epmc_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    epmc_hardware_interface::EPMCHardwareInterface, hardware_interface::SystemInterface)
