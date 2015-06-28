/*
 Copyright 2012 Barrett Technology <support@barrett.com>

 This file is part of libbarrett_ros.

 This version of libbarrett_ros is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
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
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/operations.hpp>
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

#include <realtime_tools/realtime_clock.h>
#include <libbarrett_ros/TaskSet.h> // for TaskSet
#include <libbarrett_ros/BarrettRobotHW.h>
#include <libbarrett_ros/ForceTorqueSensorHW.h>
#include <libbarrett_ros/HandHW.h>
#include <libbarrett_ros/WamHW.h>
#include <libbarrett_ros/params.h>
#include <libbarrett_ros/BusInfo.h>
#include <libbarrett_ros/Bus.h>
#include <libbarrett_ros/Schedule.h>

using barrett::ProductManager;
using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::BarrettRobotHW;
using libbarrett_ros::ForceTorqueSensorHW;
using libbarrett_ros::Schedule;
using libbarrett_ros::WamHW;
using libbarrett_ros::BusInfo;
using libbarrett_ros::get_value;
using libbarrett_ros::get_or_throw;
using libbarrett_ros::get_or_default;
using libbarrett_ros::Task;
using libbarrett_ros::TaskSet;
using libbarrett_ros::Cycle;
using libbarrett_ros::CycleInfo;
using libbarrett_ros::ScheduleInfo;
using libbarrett_ros::Bus;
using libbarrett_ros::InitializeBus;
using XmlRpc::XmlRpcValue;

int main(int argc, char **argv)
{
  using ::boost::make_shared;
  using ::boost::shared_ptr;

  // TODO: What does this do?
  barrett::installExceptionHandler();

  // Initialize ROS.
  ros::init(argc, argv, "barrett_ros");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Read parameters.
  // TODO: Is this the right way to get the private parameter namespace?
  XmlRpc::XmlRpcValue root_xmlrpc;
  ros::param::get("~", root_xmlrpc);

  std::vector<BusInfo> bus_infos;
  try {
    bus_infos = get_or_throw<std::vector<BusInfo> >(root_xmlrpc, "buses");
    ROS_INFO_STREAM("Found" << bus_infos.size() << " communication buses.");
  } catch (std::runtime_error const &e) {
    ROS_FATAL("Failed loading parameters: %s", e.what());
    return 1;
  }

  bool const is_realtime
    = get_or_default<bool>(root_xmlrpc, "realtime", false);
  if (is_realtime) {
    ROS_FATAL("Realtime support is not yet implemented.");
    return 1;
  }

  // Default to a 500 Hz conrol frequency on a 1 Mbps CAN bus.
  uint_fast32_t const control_period
    = get_or_default<int>(root_xmlrpc, "control_period", 2000);
  uint_fast32_t const bus_freq
    = get_or_default<int>(root_xmlrpc, "bus_frequency", 1e6);

  // Initialize the communication buses.
  std::vector<Bus> buses(bus_infos.size());
  for (size_t i = 0; i < bus_infos.size(); ++i) {
    BusInfo const &bus_info = bus_infos[i];
    Bus &bus = buses[i];

    ROS_INFO_STREAM("Initializing communication bus " << i << " of "
                    << bus_infos.size() << ".");
    try {
      InitializeBus(bus_info, is_realtime, &bus);
    } catch (std::runtime_error const &e) {
      ROS_FATAL_STREAM("Failed initializating communication bus "
                       << (i + 1) << ": " << e.what());
      return 1;
    }

    // Check bus utilization to, hopefully, avoid a heartbeat fault.
    double const utilization
      = bus.schedule.GetUtilization(control_period, bus_freq);

    if (utilization > bus_info.utilization_error) {
      ROS_FATAL(
        "Communication bus %zu has bus utilization of %.2f%%; exceeds"
        " threshold of %.2f%%.",
        i + 1, 100 * utilization, 100 * bus_info.utilization_error);
      // TODO: Do we need to do any cleanup here?
      return 1;
    } else if (utilization > bus_info.utilization_warn) {
      ROS_WARN(
        "Communication bus %zu has bus utilization of %.2f%%; exceeds"
        " threshold of %.2f%%.",
        i + 1, 100 * utilization, 100 * bus_info.utilization_warn);
    }
  }

  ROS_INFO("Initializing the RobotHW interface.");
  BarrettRobotHW robot;

  BOOST_FOREACH (Bus const &bus, buses) {
    BOOST_FOREACH (boost::shared_ptr<BarrettBaseHW> const &hw, bus.hardware) {
      robot.add(hw);
    }
  }

  ROS_INFO("Activating hardware.");
  robot.initialize();

  ROS_INFO("Initializing the ControllerManager.");
  controller_manager::ControllerManager cm(&robot);

  // Start the control loop. Default to 500 Hz.
  // TODO: ros::Rate and ros::Time are likely not realtime-safe.
  ROS_INFO("Entering control loop.");

  ros::Duration const period(0, control_period * 1e3);
  ros::Rate r(500);

  while (ros::ok()) {
    BOOST_FOREACH (Bus &bus, buses) {
      bus.schedule.RunPreControl();
    }

    ros::Time const now(::barrett::highResolutionSystemTime());
    cm.update(now, period);

    BOOST_FOREACH (Bus &bus, buses) {
      bus.schedule.RunPostControl();
    }

    r.sleep();
  }

  ROS_INFO("Exited control loop.");

  // Idle the hardware to avoid triggering a heartbeat fault.
  ROS_INFO("Idling hardware.");
  robot.halt();

  return 0;
}
