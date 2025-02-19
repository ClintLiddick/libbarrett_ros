#ifndef LIBBARRETT_ROS_BARRETTBASEHW_H_
#define LIBBARRETT_ROS_BARRETTBASEHW_H_
#include <vector>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_interface.h>
#include <libbarrett_ros/Task.h>

namespace libbarrett_ros {

struct BarrettInterfaces {
  hardware_interface::JointStateInterface jointstate_interface;
  hardware_interface::EffortJointInterface jointeffort_interface;
  hardware_interface::ForceTorqueSensorInterface forcetorque_interface;
  hardware_interface::ImuSensorInterface imu_interface;
  transmission_interface::TransmissionInterface<
    transmission_interface::ActuatorToJointStateHandle> transmission_interface;

  void registerAll(hardware_interface::RobotHW &hardware)
  {
    hardware.registerInterface(&jointstate_interface);
    hardware.registerInterface(&jointeffort_interface);
    hardware.registerInterface(&forcetorque_interface);
    hardware.registerInterface(&imu_interface);
    hardware.registerInterface(&transmission_interface);
  }
};

class BarrettBaseHW : public ::hardware_interface::RobotHW {
public:
  virtual ~BarrettBaseHW()
  {
  }

  virtual std::vector<Task *> const &tasks() const = 0;

  virtual std::string const &name() const = 0;

  virtual void halt() = 0;

  virtual void registerHandles(BarrettInterfaces &interfaces) = 0;
};

}

#endif
