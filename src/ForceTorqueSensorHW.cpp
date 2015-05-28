#include <libbarrett_ros/ForceTorqueSensorHW.h>
#include <barrett/systems/wam.h>

using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::ForceTorqueSensorHW;

int const ForceTorqueSensorHW::STATE_REQUEST_SENT = 0;
int const ForceTorqueSensorHW::STATE_RECEIVED_FORCE = 1;
int const ForceTorqueSensorHW::STATE_RECEIVED_TORQUE = 2;
int const ForceTorqueSensorHW::STATE_IDLE = (
    ForceTorqueSensorHW::STATE_RECEIVED_FORCE
  | ForceTorqueSensorHW::STATE_RECEIVED_TORQUE
);

ForceTorqueSensorHW::ForceTorqueSensorHW(
  ::barrett::ForceTorqueSensor *forcetorque_sensor,
  std::string const &name, std::string const &frame_id
)
  : name_(name)
  , frame_id_(frame_id)
  , force_(0.)
  , torque_(0.)
  , sensor_(forcetorque_sensor)
  , state_(STATE_IDLE)
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
}

void ForceTorqueSensorHW::receiveCritical()
{
}

void ForceTorqueSensorHW::requestOther()
{
  using barrett::Puck;

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);
  int ret = 0;
  
  ret = Puck::sendGetPropertyRequest(puck->getBus(), puck->getId(), propId);
  if (ret != 0) {
    throw std::runtime_error("Failed to send request to force/torque sensor.");
  }
  state_ = STATE_REQUEST_SENT;
}

void ForceTorqueSensorHW::receiveOther()
{
  using barrett::Puck;

  static bool const realtime = false;
  static bool const blocking = false;

  typedef barrett::ForceTorqueSensor::ForceParser ForceParser;
  typedef barrett::ForceTorqueSensor::TorqueParser TorqueParser;

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);

  // Receive the force message.
  if (!(state_ & STATE_RECEIVED_FORCE)) {
    int const ret = Puck::receiveGetPropertyReply<ForceParser>(
      puck->getBus(), puck->getId(), propId, &force_, blocking, realtime);

    if (ret == 0) {
      state_ |= STATE_RECEIVED_FORCE;
    } else if (ret == 2) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }

	// Receive the torque message.
  if (!(state_ & STATE_RECEIVED_TORQUE)) {
    int const ret = Puck::receiveGetPropertyReply<TorqueParser>(
      puck->getBus(), puck->getId(), propId, &torque_, blocking, realtime);

    if (ret == 0) {
      state_ |= STATE_RECEIVED_TORQUE;
    } else if (ret == 2) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }
}

void ForceTorqueSensorHW::write()
{
}

void ForceTorqueSensorHW::halt()
{
}
