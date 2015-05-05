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
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
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

using ::libbarrett_ros::BarrettRobotHW;
using ::libbarrett_ros::ForceTorqueSensorHW;
using ::libbarrett_ros::WamHW;

int main(int argc, char **argv)
{
  BarrettRobotHW robot;

  // Initialize ROS.
  ::ros::init(argc, argv, "libbarrett_ros");
  ::ros::NodeHandle nh;
  ::ros::AsyncSpinner spinner(2);
  spinner.start();

  // Read parameters.
  bool wait_for_shift_activate = true;
  bool prompt_on_zeroing = true;

  nh.getParam("wait_for_shift_activate", wait_for_shift_activate);
  nh.getParam("prompt_on_zeroing", prompt_on_zeroing);

  // Initialize libbarrett.
  ::barrett::installExceptionHandler();
  ::barrett::ProductManager pm;

  // TODO: Is this the right initialization order?
  pm.waitForWam(prompt_on_zeroing);
  pm.wakeAllPucks();
  pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

  // TODO: Don't hard-code joint names.
  if (pm.foundWam7()) {
    ROS_INFO("Found a 7-DOF WAM.");
    robot.add(
      boost::make_shared<WamHW<7> >(
        pm.getWam7(wait_for_shift_activate)));
  } else if (pm.foundWam4()) {
    ROS_INFO("Found a 4-DOF WAM.");
    robot.add(
      boost::make_shared<WamHW<4> >(
        pm.getWam4(wait_for_shift_activate)));
  } else if (pm.foundWam3()) {
    ROS_INFO("Found a 3-DOF WAM.");
    robot.add(
      boost::make_shared<WamHW<3> >(
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
      boost::make_shared<ForceTorqueSensorHW>(
        pm.getForceTorqueSensor(), "ft_sensor", "ft_sensor_frame"));
  }

  // Initialize ros_control.
  robot.initialize();
  ::controller_manager::ControllerManager cm(&robot);

  // Start the control loop.
  ::barrett::systems::RealTimeExecutionManager *execution_manager
    = pm.getExecutionManager();
  ros::Duration const period(execution_manager->getPeriod());

  // TODO: Can we run this inside the RealTimeExecutionManager?
  while (ros::ok()) {
    robot.read();

    ros::Time const now(::barrett::highResolutionSystemTime());
    cm.update(now, period);

    robot.write();
  }

  return 0;
}
