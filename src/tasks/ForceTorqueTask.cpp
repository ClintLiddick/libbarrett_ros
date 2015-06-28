#include <barrett/products/force_torque_sensor.h>
#include <libbarrett_ros/tasks/ForceTorqueTask.h>

namespace libbarrett_ros {

int const ForceTorqueTask::STATE_REQUEST_SENT = 0;
int const ForceTorqueTask::STATE_RECEIVED_FORCE = 1;
int const ForceTorqueTask::STATE_RECEIVED_TORQUE = 2;
int const ForceTorqueTask::STATE_IDLE = (
    ForceTorqueTask::STATE_RECEIVED_FORCE
  | ForceTorqueTask::STATE_RECEIVED_TORQUE
);

ForceTorqueTask::ForceTorqueTask(barrett::Puck *puck, bool realtime,
                                 cf_type *force, ct_type *torque)
    : puck_(puck)
    , realtime_(realtime)
    , state_(STATE_IDLE)
    , force_(force)
    , torque_(torque)
{
  using barrett::Puck;

  prop_id_ = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);
}

ForceTorqueTask::~ForceTorqueTask()
{
}

std::string const &ForceTorqueTask::name() const
{
    static std::string const name = "ft_wrench";
    return name;
}

uint_fast32_t ForceTorqueTask::request_bits() const
{
  // 47 bit header + 1 byte data
  // TODO: The Barrett docs imply that there are three payload bytes.
  return 47 + 8;
}

uint_fast32_t ForceTorqueTask::receive_bits() const
{
  //   47 bit header + 6 bytes data for force
  // + 47 bit header + 6 bytes data for torque
  // + 1 byte (on the torque frame) if saturated
  return 2 * (47 + 6 * 8) + 8;
}

uint_fast32_t ForceTorqueTask::write_bits() const
{
}

void ForceTorqueTask::Request()
{
  using barrett::Puck;
  using std::runtime_error;

  if (state_ != STATE_IDLE) {
    throw runtime_error("There is another pending force/torque request.");
  }

  int const ret = Puck::sendGetPropertyRequest(
    puck_->getBus(), puck_->getId(), prop_id_);

  if (ret != 0) {
    throw runtime_error("Failed to send request to force/torque sensor.");
  }

  state_ = STATE_REQUEST_SENT;
}

void ForceTorqueTask::Receive(bool blocking)
{
  using barrett::Puck;

  typedef barrett::ForceTorqueSensor::ForceParser ForceParser;
  typedef barrett::ForceTorqueSensor::TorqueParser TorqueParser;

  // Receive the force message.
  if (!(state_ & STATE_RECEIVED_FORCE)) {
    int const ret = Puck::receiveGetPropertyReply<ForceParser>(
      puck_->getBus(), puck_->getId(), prop_id_, force_, blocking, realtime_);

    if (ret == 0) {
      state_ |= STATE_RECEIVED_FORCE;
    } else if (ret != 1) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }

	// Receive the torque message.
  if (!(state_ & STATE_RECEIVED_TORQUE)) {
    int const ret = Puck::receiveGetPropertyReply<TorqueParser>(
      puck_->getBus(), puck_->getId(), prop_id_, torque_,
      blocking, realtime_);

    if (ret == 0) {
      state_ |= STATE_RECEIVED_TORQUE;
    } else if (ret != 1) {
      throw std::runtime_error("Failed to receive torque.");
    }
  }
}

void ForceTorqueTask::Write()
{
  // do nothing
}

} // namespace libbarrett_ros
