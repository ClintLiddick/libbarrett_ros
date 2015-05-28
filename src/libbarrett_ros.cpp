/*
 Copyright 2012 Barrett Technology <support@barrett.com>

 This file is part of libbarrett_ros.

 This version of libbarrett_ros is free software: you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.

 This version of libbarrett_ros is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this version of libbarrett_ros.  If not, see
 <http://www.gnu.org/licenses/>.

 Barrett Technology holds all copyrights on libbarrett_ros. As the sole
 copyright holder, Barrett reserves the right to release future versions
 of libbarrett_ros under a different license.

 File: libbarrett_ros.cpp
 Date: 15 October, 2014
 Author: Hariharasudan Malaichamee
 */
#include <vector>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <barrett/config.h>
#include <barrett/exception.h>
#include <barrett/systems.h>

#include "ros/ros.h"
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <barrett/products/product_manager.h>
#include <boost/thread.hpp>
#include <barrett/systems/real_time_execution_manager.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <libbarrett_ros/BarrettRobotHW.h>
#include <libbarrett_ros/ForceTorqueSensorHW.h>
#include <libbarrett_ros/WamHW.h>

using ::barrett::ProductManager;
using ::libbarrett_ros::BarrettRobotHW;
using ::libbarrett_ros::ForceTorqueSensorHW;
using ::libbarrett_ros::WamHW;
using ::XmlRpc::XmlRpcValue;

static std::vector<std::string> get_configuration_paths(
    XmlRpcValue &configs_xmlrpc
  )
{
  using ::boost::format;
  using ::boost::str;
  using ::boost::filesystem::is_directory;
  using ::boost::filesystem::is_regular_file;

  std::vector<std::string> configs;

  switch (configs_xmlrpc.getType()) {
  case XmlRpcValue::TypeArray:
    ROS_INFO_STREAM("Found " << configs_xmlrpc.size() << " configuration file(s).");

    for (int i = 0; i < configs_xmlrpc.size(); ++i) {
      XmlRpcValue &config_xmlrpc = configs_xmlrpc[i];

      if (config_xmlrpc.getType() == XmlRpcValue::TypeString) {
        configs.push_back(config_xmlrpc);
      } else {
        throw std::runtime_error("Configuration file path is not a string.");
      }
    }
    break;

  case XmlRpcValue::TypeString:
    ROS_INFO("Found one configuration file.");
    configs.push_back(configs_xmlrpc);
    break;

  case XmlRpcValue::TypeInvalid:
    ROS_INFO("Using default configuration file.");
    configs.push_back(::barrett::EtcPathRelative("default.conf"));
    break;

  default:
    throw std::runtime_error("Invalid configurations.");
  }

  // Run some sanity checks to catch common mistakes.
  for (size_t i = 0; i < configs.size(); ++i) {
    std::string const &config_path = configs[i];

    if (is_directory(config_path)) {
      throw std::runtime_error(str(
        format("Configuration path '%s' is a directory, but should be a file."
               " Did you forget to include 'default.conf'?") % config_path));
    } else if (!is_regular_file(config_path)) {
      throw std::runtime_error(str(
        format("Configuration path '%s' is not a file.") % config_path));
    }
  }

  return configs;
}

static void initialize_product_manager(
    ProductManager &pm,
    BarrettRobotHW &robot
  )
{
  using ::boost::make_shared;

  // TODO: Can we safely set both of these to false?
  static bool const prompt_on_zeroing = true;
  static bool const wait_for_shift_activate = true;

  // TODO: Can we defer some of this to later?
  pm.waitForWam(prompt_on_zeroing);
  pm.wakeAllPucks();

  // TODO: What does this do?
  pm.getSafetyModule()->waitForMode(::barrett::SafetyModule::IDLE);

  // TODO: Don't hard-code joint names.
  if (pm.foundWam7()) {
    ROS_INFO("Found a 7-DOF WAM.");
    robot.add(
      make_shared<WamHW<7> >(
        pm.getWam7(wait_for_shift_activate)));
  } else if (pm.foundWam4()) {
    ROS_INFO("Found a 4-DOF WAM.");
    robot.add(
      make_shared<WamHW<4> >(
        pm.getWam4(wait_for_shift_activate)));
  } else if (pm.foundWam3()) {
    ROS_INFO("Found a 3-DOF WAM.");
    robot.add(
      make_shared<WamHW<3> >(
        pm.getWam3(wait_for_shift_activate)));
  }

  if (pm.foundHand()) {
    ROS_INFO("Found a BarrettHand.");
    ::barrett::Hand *ft = pm.getHand();
    // TODO: Implement a BarrettHand hardware interface.
    ROS_WARN("The BarrettHand is not yet supported.");
  }

  if (pm.foundForceTorqueSensor()) {
    ROS_INFO("Found a force/torque sensor.");
    // TODO: Don't hard-code the name and frame_id.
    robot.add(
      make_shared<ForceTorqueSensorHW>(
        pm.getForceTorqueSensor(), "ft_sensor", "ft_sensor_frame"));
  }
}

int main(int argc, char **argv)
{
  using ::boost::make_shared;
  using ::boost::shared_ptr;

  ::barrett::installExceptionHandler();

  // Initialize ROS.
  ::ros::init(argc, argv, "barrett_ros");
  ::ros::NodeHandle nh;
  ::ros::AsyncSpinner spinner(2);
  spinner.start();

  // Read parameters.
  ::XmlRpc::XmlRpcValue configs_xmlrpc;
  ::std::vector<std::string> config_paths;

  ::ros::NodeHandle nh_priv("~");
  nh_priv.getParam("configurations", configs_xmlrpc);

  try {
    config_paths = get_configuration_paths(configs_xmlrpc);
    ROS_INFO_STREAM("Loading " << config_paths.size() << " config files.");
  } catch (std::runtime_error const &e) {
    ROS_FATAL("Failed loading configurations: %s", e.what());
    return 1;
  }

  // Initialize libbarrett.
  BarrettRobotHW robot;
  std::vector<shared_ptr<ProductManager> > product_managers;
  product_managers.reserve(config_paths.size());

  BOOST_FOREACH (std::string const &config_path, config_paths) {
    ROS_INFO_STREAM("Loading configuration file '" << config_path << "'.");

    shared_ptr<ProductManager> const product_manager
      = make_shared<ProductManager>(config_path.c_str());

    initialize_product_manager(*product_manager, robot);
    product_managers.push_back(product_manager);

    product_manager->getExecutionManager()->stop();
  }

  // Initialize ros_control.
  robot.initialize();
  ::controller_manager::ControllerManager cm(&robot);

  // Start the control loop. Default to 500 Hz.
  // TODO: Figure out threading and real-time safety.
  ROS_INFO("Entering control loop.");

  ros::Duration const period(0, 500000000);
  ros::Rate r(500);

  while (ros::ok()) {
    robot.requestCritical();
    robot.receiveOther();
    robot.receiveCritical();
    robot.requestOther();

    ros::Time const now(::barrett::highResolutionSystemTime());
    cm.update(now, period);

    robot.write();

    r.sleep();
  }

  ROS_INFO("Exited control loop.");

  // Idle the hardware to avoid triggering a heartbeat fault.
  robot.halt();

  return 0;
}
