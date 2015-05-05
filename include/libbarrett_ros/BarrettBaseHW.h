#ifndef LIBBARRETT_ROS_BARRETTBASEHW_H_
#define LIBBARRETT_ROS_BARRETTBASEHW_H_
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace libbarrett_ros {

struct BarrettInterfaces {
  ::hardware_interface::JointStateInterface jointstate_interface;
  ::hardware_interface::EffortJointInterface jointeffort_interface;
  ::hardware_interface::ForceTorqueSensorInterface forcetorque_interface;

  void registerAll(::hardware_interface::RobotHW &hardware)
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

}

#endif
