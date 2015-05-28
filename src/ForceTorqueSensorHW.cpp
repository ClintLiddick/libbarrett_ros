#include <libbarrett_ros/ForceTorqueSensorHW.h>

using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::ForceTorqueSensorHW;

ForceTorqueSensorHW::ForceTorqueSensorHW(
  ::barrett::ForceTorqueSensor *forcetorque_sensor,
  std::string const &name, std::string const &frame_id
)
  : name_(name)
  , frame_id_(frame_id)
  , force_(0.)
  , torque_(0.)
  , sensor_(forcetorque_sensor)
{
}

ForceTorqueSensorHW::~ForceTorqueSensorHW()
{
}

void ForceTorqueSensorHW::registerHandles(BarrettInterfaces &interfaces)
{
  interfaces.forcetorque_interface.registerHandle(
    hardware_interface::ForceTorqueSensorHandle(
      name_, frame_id_, &force_[0], &torque_[0]
    )
  );

  // TODO: Also register an IMU for acceleration.
}

void ForceTorqueSensorHW::requestCritical()
{
  // TODO: Split this into read() and update().
  force_ = sensor_->getForce();
  torque_ = sensor_->getTorque();
}

void ForceTorqueSensorHW::receiveCritical()
{
}

void ForceTorqueSensorHW::requestOther()
{
}

void ForceTorqueSensorHW::receiveOther()
{
}

void ForceTorqueSensorHW::write()
{
}

void ForceTorqueSensorHW::halt()
{
}
