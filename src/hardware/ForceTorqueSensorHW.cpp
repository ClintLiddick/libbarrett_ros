#include <libbarrett_ros/hardware/ForceTorqueSensorHW.h>
#include <barrett/systems/wam.h>

using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::ForceTorqueSensorHW;
using libbarrett_ros::Task;

ForceTorqueSensorHW::ForceTorqueSensorHW(
  ::barrett::ForceTorqueSensor *forcetorque_sensor,
  bool realtime,
  std::string const &forcetorque_name,
  std::string const &accelerometer_name,
  std::string const &frame_id
)
  : tasks_(2)
  , ft_task_(forcetorque_sensor->getPuck(), realtime, &force_, &torque_)
  , accel_task_(forcetorque_sensor->getPuck(), realtime, &accel_)
  , forcetorque_name_(forcetorque_name)
  , accelerometer_name_(accelerometer_name)
  , frame_id_(frame_id)
  , force_(0.)
  , torque_(0.)
  , sensor_(forcetorque_sensor)
  , realtime_(realtime)
{
  tasks_[0] = &ft_task_;
  tasks_[1] = &accel_task_;
}

ForceTorqueSensorHW::~ForceTorqueSensorHW()
{
}

std::vector<Task *> const &ForceTorqueSensorHW::tasks() const
{
  return tasks_;
}

void ForceTorqueSensorHW::registerHandles(BarrettInterfaces &interfaces)
{
  interfaces.forcetorque_interface.registerHandle(
    hardware_interface::ForceTorqueSensorHandle(
      forcetorque_name_, frame_id_, &force_[0], &torque_[0]
    )
  );

  hardware_interface::ImuSensorHandle::Data imu_interface_data;
  imu_interface_data.name = accelerometer_name_;
  imu_interface_data.frame_id = frame_id_;
  imu_interface_data.linear_acceleration = &accel_[0];

  interfaces.imu_interface.registerHandle(
    hardware_interface::ImuSensorHandle(imu_interface_data)
  );
}

void ForceTorqueSensorHW::halt()
{
}
