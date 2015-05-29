#include <libbarrett_ros/ForceTorqueSensorHW.h>
#include <barrett/systems/wam.h>

using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::ForceTorqueSensorHW;

int const ForceTorqueSensorHW::ACCEL_REQUEST_SENT = 0;
int const ForceTorqueSensorHW::ACCEL_IDLE = 1;

int const ForceTorqueSensorHW::FORCETORQUE_REQUEST_SENT = 0;
int const ForceTorqueSensorHW::FORCETORQUE_RECEIVED_FORCE = 1;
int const ForceTorqueSensorHW::FORCETORQUE_RECEIVED_TORQUE = 2;
int const ForceTorqueSensorHW::FORCETORQUE_IDLE = (
    ForceTorqueSensorHW::FORCETORQUE_RECEIVED_FORCE
  | ForceTorqueSensorHW::FORCETORQUE_RECEIVED_TORQUE
);

ForceTorqueSensorHW::ForceTorqueSensorHW(
  ::barrett::ForceTorqueSensor *forcetorque_sensor,
  std::string const &forcetorque_name,
  std::string const &accelerometer_name,
  std::string const &frame_id
)
  : forcetorque_name_(forcetorque_name)
  , accelerometer_name_(accelerometer_name)
  , frame_id_(frame_id)
  , force_(0.)
  , torque_(0.)
  , sensor_(forcetorque_sensor)
  , realtime_(false)
  , accel_state_(ACCEL_IDLE)
  , ft_state_(FORCETORQUE_IDLE)
{
}

ForceTorqueSensorHW::~ForceTorqueSensorHW()
{
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

void ForceTorqueSensorHW::requestCritical()
{
}

void ForceTorqueSensorHW::receiveCritical()
{
}

void ForceTorqueSensorHW::requestOther()
{
  requestForceTorque();
  requestAccel();
}

void ForceTorqueSensorHW::receiveOther()
{
  receiveForceTorque(false);
  receiveAccel(false);
}

void ForceTorqueSensorHW::write()
{
}

void ForceTorqueSensorHW::halt()
{
}

void ForceTorqueSensorHW::requestForceTorque()
{
  using barrett::Puck;

  if (ft_state_ != FORCETORQUE_IDLE) {
    throw std::runtime_error("There is another pending force/torque request.");
  }

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);
  int const ret = Puck::sendGetPropertyRequest(
    puck->getBus(), puck->getId(), propId);

  if (ret != 0) {
    throw std::runtime_error("Failed to send request to force/torque sensor.");
  }

  ft_state_ = FORCETORQUE_REQUEST_SENT;
}

void ForceTorqueSensorHW::receiveForceTorque(bool blocking)
{
  using barrett::Puck;

  typedef barrett::ForceTorqueSensor::ForceParser ForceParser;
  typedef barrett::ForceTorqueSensor::TorqueParser TorqueParser;

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);

  // Receive the force message.
  if (!(ft_state_ & FORCETORQUE_RECEIVED_FORCE)) {
    int const ret = Puck::receiveGetPropertyReply<ForceParser>(
      puck->getBus(), puck->getId(), propId, &force_, blocking, realtime_);

    if (ret == 0) {
      ft_state_ |= FORCETORQUE_RECEIVED_FORCE;
    } else if (ret != 1) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }

	// Receive the torque message.
  if (!(ft_state_ & FORCETORQUE_RECEIVED_TORQUE)) {
    int const ret = Puck::receiveGetPropertyReply<TorqueParser>(
      puck->getBus(), puck->getId(), propId, &torque_, blocking, realtime_);

    if (ret == 0) {
      ft_state_ |= FORCETORQUE_RECEIVED_TORQUE;
    } else if (ret != 1) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }
}

void ForceTorqueSensorHW::requestAccel()
{
  using barrett::Puck;

  if (accel_state_ != ACCEL_IDLE) {
    throw std::runtime_error("There is another pending acceleration request.");
  }

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::A, Puck::PT_ForceTorque, 0);
  int const ret = Puck::sendGetPropertyRequest(
    puck->getBus(), puck->getId(), propId);

  if (ret != 0) {
    throw std::runtime_error("Failed to send request to accelerometer.");
  }

  accel_state_ = ACCEL_REQUEST_SENT;
}

void ForceTorqueSensorHW::receiveAccel(bool blocking)
{
  using barrett::Puck;
  typedef barrett::ForceTorqueSensor::AccelParser AccelParser;

  if (accel_state_ != ACCEL_REQUEST_SENT) {
    return;
  }

  Puck *const puck = sensor_->getPuck();
  int const propId = Puck::getPropertyId(Puck::A, Puck::PT_ForceTorque, 0);
  int const ret = Puck::receiveGetPropertyReply<AccelParser>(
    puck->getBus(), puck->getId(), propId, &accel_, blocking, realtime_);

  if (ret == 0) {
    accel_state_ = ACCEL_IDLE;
  } else if (ret != 1) {
    throw std::runtime_error("Failed to receive acceleration.");
  }
}
