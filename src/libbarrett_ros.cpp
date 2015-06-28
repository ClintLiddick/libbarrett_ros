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
using XmlRpc::XmlRpcValue;

struct Bus {
  boost::shared_ptr<ProductManager> product_manager;
  std::vector<boost::shared_ptr<BarrettBaseHW> > hardware;
  boost::shared_ptr<Schedule> schedule;
};

template <class T, size_t N>
static boost::array<T, N> to_array(std::vector<T> const &v)
{
  using boost::format;
  using boost::str;

  if (v.size() != N) {
    throw std::runtime_error(str(
      format("Expected array to be of length %d; got %d.") % N % v.size()));
  }

  boost::array<T, N> output;
  for (size_t i = 0; i < N; ++i) {
    output[i] = v[i];
  }
  return output;
}

static boost::shared_ptr<Schedule> CreateSchedule(
  TaskSet const &tasks, ScheduleInfo const &schedule_info)
{
  using boost::make_shared;

  std::vector<Cycle> cycles(schedule_info.cycles.size());

  for (size_t icycle = 0; icycle < schedule_info.cycles.size(); ++icycle) {
    CycleInfo const &cycle_info = schedule_info.cycles[icycle];
    Cycle &cycle = cycles[icycle];

    BOOST_FOREACH (std::string const &task_name, cycle_info.required_tasks) {
      cycle.AddTask(tasks.GetTask(task_name), true);
    }

    BOOST_FOREACH (std::string const &task_name, cycle_info.optional_tasks) {
      cycle.AddTask(tasks.GetTask(task_name), false);
    }
  }

  return boost::make_shared<Schedule>(cycles);
}

static void InitializeBus(BusInfo const &bus_info, bool is_realtime, Bus *bus)
{
  using boost::make_shared;

  // TODO: Can we safely set both of these to false?
  static bool const prompt_on_zeroing = true;
  static bool const wait_for_shift_activate = true;

  // Initialize the ProductManager from configuration files.
  std::string const &config_path = bus_info.configuration_path;
  ROS_INFO_STREAM("Loading configuration file '" << config_path << "'.");
  bus->product_manager = make_shared<ProductManager>(config_path.c_str());

  // Stop the libbarrett thread. We'll handle everything oruselves.
  ProductManager &pm = *bus->product_manager;
  pm.getExecutionManager()->stop();

  // TODO: Can we defer some of this to later?
  pm.waitForWam(prompt_on_zeroing);
  pm.wakeAllPucks();

  // TODO: What does this do?
  pm.getSafetyModule()->waitForMode(::barrett::SafetyModule::IDLE);

  bus->hardware.reserve(3);

  if (pm.foundWam7()) {
    ROS_INFO("Found a 7-DOF WAM.");
    boost::array<std::string, 7> const wam_joint_names
      = to_array<std::string, 7>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<7> >(
        pm.getWam7(wait_for_shift_activate), is_realtime, &wam_joint_names));
  } else if (pm.foundWam4()) {
    ROS_INFO("Found a 4-DOF WAM.");
    boost::array<std::string, 4> const wam_joint_names
      = to_array<std::string, 4>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<4> >(
        pm.getWam4(wait_for_shift_activate), is_realtime, &wam_joint_names));
  } else if (pm.foundWam3()) {
    ROS_INFO("Found a 3-DOF WAM.");
    boost::array<std::string, 3> const wam_joint_names
      = to_array<std::string, 3>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<3> >(
        pm.getWam3(wait_for_shift_activate), is_realtime, &wam_joint_names));
  }

  if (pm.foundHand()) {
    ROS_INFO("Found a BarrettHand.");
    ::barrett::Hand *ft = pm.getHand();
    // TODO: Implement a BarrettHand hardware interface.
    ROS_WARN("The BarrettHand is not yet supported.");
  }

  if (pm.foundForceTorqueSensor()) {
    ROS_INFO("Found a force/torque sensor.");
    bus->hardware.push_back(
      make_shared<ForceTorqueSensorHW>(
        pm.getForceTorqueSensor(), is_realtime,
        bus_info.forcetorque_wrench_name,
        bus_info.forcetorque_accel_name,
        bus_info.forcetorque_frame_id));
  }

  // Aggregate all of the tasks on this bus.
  TaskSet tasks;
  BOOST_FOREACH (boost::shared_ptr<BarrettBaseHW> const &hw, bus->hardware) {
    BOOST_FOREACH (Task *task, hw->tasks()) {
      tasks.AddTask(task);
    }
  }

  bus->schedule = CreateSchedule(tasks, bus_info.schedule);
}

int main(int argc, char **argv)
{
  using ::boost::make_shared;
  using ::boost::shared_ptr;

  // TODO: These should be read from parameters.
  uint_fast32_t const control_period = 2000; // us = 500 Hz
  uint_fast32_t const bus_freq = 1e6; // bps = 1 Mbps
  bool const is_realtime = false;

  ::barrett::installExceptionHandler();

  // Initialize ROS.
  ::ros::init(argc, argv, "barrett_ros");
  ::ros::NodeHandle nh;
  ::ros::AsyncSpinner spinner(2);
  spinner.start();

  // Read parameters.
  XmlRpc::XmlRpcValue root_xmlrpc;
  ros::param::get("~", root_xmlrpc);

  std::vector<BusInfo> bus_infos;
  try {
    // TODO: This should operate on teh root XmlRpcValue, not "buses".
    bus_infos = get_or_throw<std::vector<BusInfo> >(root_xmlrpc, "buses");
    ROS_INFO_STREAM("Found" << bus_infos.size() << " communication buses.");
  } catch (std::runtime_error const &e) {
    ROS_FATAL("Failed loading parameters: %s", e.what());
    return 1;
  }

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
      = bus.schedule->GetUtilization(control_period, bus_freq);

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

  ros::Duration const period(0, 500000000);
  ros::Rate r(500);

  while (ros::ok()) {
    BOOST_FOREACH (Bus const &bus, buses) {
      bus.schedule->RunPreControl();
    }

    ros::Time const now(::barrett::highResolutionSystemTime());
    cm.update(now, period);

    BOOST_FOREACH (Bus const &bus, buses) {
      bus.schedule->RunPostControl();
    }

    r.sleep();
  }

  ROS_INFO("Exited control loop.");

  // Idle the hardware to avoid triggering a heartbeat fault.
  ROS_INFO("Idling hardware.");
  robot.halt();

  return 0;
}
