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
#include <boost/array.hpp>
#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/force_torque_sensor.h>

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

class BarrettBaseHW : public ::hardware_interface::RobotHW {
public:
  virtual ~BarrettBaseHW()
  {
  }

  virtual void read() = 0;
  virtual void write() = 0;
};

template <size_t DOF>
class WamHW : public BarrettBaseHW {
public:
  WamHW(::barrett::systems::Wam<DOF> *wam)
    : state_position_(0.)
    , state_velocity_(0.)
    , state_effort_(0.)
    , command_effort_(0.)
    , wam_(wam)
    , llwam_(&wam_->getLowLevelWam())
  {
    // TODO: Make this configurable.
    joint_names_[0] = "j1";
    joint_names_[2] = "j2";
    joint_names_[3] = "j3";
    joint_names_[3] = "j4";
    joint_names_[4] = "j5";
    joint_names_[5] = "j6";
    joint_names_[6] = "j7";
  }

  void registerAllInterfaces()
  {
    for (size_t i = 0; i < DOF; ++i) {
      // Create the JointStateHandle.
      handle_jointstate_[i] = ::hardware_interface::JointStateHandle(
        joint_names_[i],
        &state_position_[i], &state_velocity_[i], &state_effort_[i]
      );
      jointstate_interface_.registerHandle(handle_jointstate_[i]);

      // Create the corresponding EffortJointInterface.
      ::hardware_interface::JointHandle jointeffort_handle(
        handle_jointstate_[i], &command_effort_[i]
      );
      jointeffort_interface_.registerHandle(jointeffort_handle);
    }

    registerInterface(&jointstate_interface_);
    registerInterface(&jointeffort_interface_);
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
  boost::array<hardware_interface::JointStateHandle, DOF> handle_jointstate_;

  hardware_interface::JointStateInterface jointstate_interface_;
  hardware_interface::EffortJointInterface jointeffort_interface_;

  jp_type state_position_;
  jv_type state_velocity_;
  jt_type state_effort_;
  jt_type command_effort_;

  ::barrett::systems::Wam<DOF> *wam_;
  ::barrett::LowLevelWam<DOF> *llwam_;

  DISALLOW_COPY_AND_ASSIGN(WamHW);
};

int main(int argc, char **argv)
{
  // Initialize ROS.
  ::ros::init(argc, argv, "libbarrett_ros");
  ::ros::NodeHandle nh;
  ::ros::AsyncSpinner spinner(2);
  spinner.start();

  bool wait_for_shift_activate = true;
  bool prompt_on_zeroing = true;
  nh.getParam("wait_for_shift_activate", wait_for_shift_activate);
  nh.getParam("prompt_on_zeroing", prompt_on_zeroing);

#if 0
	// Initialize libbarrett.
	::barrett::installExceptionHandler();
	::barrett::ProductManager pm;

	pm.waitForWam(prompt_on_zeroing);
	pm.wakeAllPucks();

  if (pm.foundWam7()) {
    // TODO: This also takes a config path.
    ::barrett::systems::Wam<7> *wam = pm.getWam7(wait_for_shift_activate);
    wam->gravityCompensate();

    WamHW<7> bhw(wam, &pm, nh);
    ::barrett::systems::connect(wam->jpOutput, bhw.input);
    wam->trackReferenceSignal(bhw.output);
  } else if (pm.foundWam4()) {
    ROS_WARN("The 4-DOF WAM is not yet supported.");
  } else if (pm.foundWam3()) {
    ROS_WARN("The 3-DOF WAM is not yet supported.");
  }

  if (pm.foundHand()) {
    ::barrett::Hand *ft = pm.getHand();
    ROS_WARN("The BarrettHand is not yet supported.");
  }

  if (pm.foundForceTorqueSensor()) {
    ::barrett::ForceTorqueSensor *ft = pm.getForceTorqueSensor();
    ROS_WARN("The Barrett force/torque sensors is not yet supported.");
  }

  // Wait for the user to press Shift-idle
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
#endif

  WamHW<7> robot(NULL);
  ros::Duration period(0.01);

  ::controller_manager::ControllerManager cm(&robot);

  while (ros::ok()) {
    robot.read();
    cm.update(ros::Time::now(), period);
    robot.write();

#if 0
    cm.update(
      static_cast<ros::Time>(highResolutionSystemTime()),
      static_cast<ros::Duration>(arm_pm->getExecutionManager()->getPeriod())
    );
#endif
  }

  ros::spin();

  return 0;
}
