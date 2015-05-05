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
#include <barrett/products/force_torque_sensor.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include "ros/ros.h"
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <barrett/products/product_manager.h>
#include <boost/thread.hpp>
#include <barrett/systems/real_time_execution_manager.h>

//#include <barrett_arm.h>
//#include <barrett_hand.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// Ros param names
const std::string robot_description = "robot_description";
const std::string tip_joint = "tip_joint";
using namespace barrett;

struct BarrettInterfaces {
  hardware_interface::JointStateInterface jointstate_interface;
  hardware_interface::EffortJointInterface jointeffort_interface;
  hardware_interface::ForceTorqueSensorInterface forcetorque_interface;

  void registerAll(hardware_interface::RobotHW &hardware)
  {
    hardware.registerInterface(&jointstate_interface);
    hardware.registerInterface(&jointeffort_interface);
    hardware.registerInterface(&forcetorque_interface);
  }
};

class BarrettBaseHW : public ::hardware_interface::RobotHW {
public:
  virtual ~BarrettBaseHW()
  {
  }

  virtual void read() = 0;
  virtual void write() = 0;

  virtual void registerHandles(BarrettInterfaces &interfaces) = 0;
};

class BarrettAggregateHW : public ::hardware_interface::RobotHW {
public:
  BarrettAggregateHW()
  {
  }

  virtual ~BarrettAggregateHW()
  {
  }

  void add(boost::shared_ptr<BarrettBaseHW> const &hardware)
  {
    hardware_.push_back(hardware);
    hardware->registerHandles(interfaces_);
  }

  void initialize()
  {
    interfaces_.registerAll(*this);
  }

  virtual void read()
  {
    for (size_t i = 0; i < hardware_.size(); ++i) {
      hardware_[i]->read();
    }
  }

  virtual void write()
  {
    for (size_t i = 0; i < hardware_.size(); ++i) {
      hardware_[i]->write();
    }
  }

private:
  BarrettInterfaces interfaces_;
  std::vector<boost::shared_ptr<BarrettBaseHW> > hardware_;
};

template <size_t DOF>
class WamHW : public BarrettBaseHW {
public:
  WamHW(::barrett::systems::Wam<DOF> *wam,
        boost::array<std::string, DOF> const *joint_names = NULL)
    : state_position_(0.)
    , state_velocity_(0.)
    , state_effort_(0.)
    , command_effort_(0.)
    , wam_(wam)
    , llwam_(&wam_->getLowLevelWam())
  {
    if (joint_names) {
      joint_names_ = *joint_names;
    } else {
      for (size_t i = 0; i < DOF; ++i) {
        joint_names_[i] = boost::str(boost::format("j%d") % i);
      }
    }
  }

  virtual ~WamHW()
  {
  }

  virtual void registerHandles(BarrettInterfaces &interfaces)
  {
    for (size_t i = 0; i < DOF; ++i) {
      ::hardware_interface::JointStateHandle jointstate_handle(
        joint_names_[i],
        &state_position_[i], &state_velocity_[i], &state_effort_[i]
      );
      interfaces.jointstate_interface.registerHandle(jointstate_handle);

      interfaces.jointeffort_interface.registerHandle(
        ::hardware_interface::JointHandle(
          jointstate_handle, &command_effort_[i]
        )
      );
    }
  }

  virtual void read()
  {
    llwam_->update();

    state_position_ = wam_->getJointPositions();
    state_velocity_ = wam_->getJointVelocities();
    state_effort_ = wam_->getJointTorques();
  }

  virtual void write()
  {
    // TODO: I'm not exactly what the difference is between "Wam",
    // "LowLevelWamWrapper", and "LowLevelWam". How do I correctly write
    // torques here that: (1) include gravity compensation and (2) prevent the
    // pucks from heartbeat faulting.
    llwam_->setTorques(command_effort_);
  }

private:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  boost::array<std::string, DOF> joint_names_;

  jp_type state_position_;
  jv_type state_velocity_;
  jt_type state_effort_;
  jt_type command_effort_;

  ::barrett::systems::Wam<DOF> *wam_;
  ::barrett::LowLevelWam<DOF> *llwam_;

  DISALLOW_COPY_AND_ASSIGN(WamHW);
};


class ForceTorqueSensorHW : public BarrettBaseHW {
public:
  ForceTorqueSensorHW(
      ::barrett::ForceTorqueSensor *forcetorque_sensor,
      std::string const &name, std::string const &frame_id)
    : name_(name)
    , frame_id_(frame_id)
    , force_(0.)
    , torque_(0.)
    , sensor_(forcetorque_sensor)
  {
  }

  virtual ~ForceTorqueSensorHW()
  {
  }

  virtual void registerHandles(BarrettInterfaces &interfaces)
  {
    // TODO: Are force_ and torque_ guaranteed to be contiguous?
    interfaces.forcetorque_interface.registerHandle(
      hardware_interface::ForceTorqueSensorHandle(
        name_, frame_id_, &force_[0], &torque_[0]
      )
    );
  }

  virtual void read()
  {
    force_ = sensor_->getForce();
    torque_ = sensor_->getForce();
  }

  virtual void write()
  {
  }

private:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  hardware_interface::ForceTorqueSensorInterface forcetorque_interface_;

  std::string name_;
  std::string frame_id_;
  cf_type force_;
  cf_type torque_;

  ::barrett::ForceTorqueSensor *sensor_;
};

int main(int argc, char **argv)
{
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
  BarrettAggregateHW robot;

	pm.waitForWam(prompt_on_zeroing);
	pm.wakeAllPucks();
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

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
    ROS_WARN("The BarrettHand is not yet supported.");
  }

  if (pm.foundForceTorqueSensor()) {
    ROS_INFO("Found a force/torque sensor.");
    robot.add(
      boost::make_shared<ForceTorqueSensorHW>(
        pm.getForceTorqueSensor(), "ft_sensor", "ft_sensor_frame"));
  }

  // Initialize ros_control.
  robot.initialize();
  ::controller_manager::ControllerManager cm(&robot);

  ros::Duration period(0.1);

  ROS_INFO("Starting control loop.");

  while (ros::ok()) {
    //robot.read();
    cm.update(ros::Time::now(), period);
    //robot.write();

#if 0
    cm.update(
      static_cast<ros::Time>(highResolutionSystemTime()),
      static_cast<ros::Duration>(arm_pm->getExecutionManager()->getPeriod())
    );
#endif

  }
  ROS_INFO("Exiting control loop.");

  return 0;
}
