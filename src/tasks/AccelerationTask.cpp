#include <barrett/products/force_torque_sensor.h>
#include <libbarrett_ros/tasks/AccelerationTask.h>

namespace libbarrett_ros {

int const AccelerationTask::STATE_REQUEST_SENT = 0;
int const AccelerationTask::STATE_IDLE = 1;

AccelerationTask::AccelerationTask(barrett::Puck *puck, bool realtime,
                                   ca_type *accel)
    : puck_(puck)
    , realtime_(realtime)
    , state_(STATE_IDLE)
    , accel_(accel)
{
  using barrett::Puck;

  prop_id_ = Puck::getPropertyId(Puck::A, Puck::PT_ForceTorque, 0);
}

AccelerationTask::~AccelerationTask()
{
}

void AccelerationTask::Request()
{
  using barrett::Puck;
  using std::runtime_error;

  if (state_ != STATE_IDLE) {
    throw runtime_error("There is another pending acceleration request.");
  }

  int const ret = Puck::sendGetPropertyRequest(
    puck_->getBus(), puck_->getId(), prop_id_);


  if (ret != 0) {
    throw runtime_error("Failed to send request to accelerometer.");
  }

  state_ = STATE_REQUEST_SENT;
}

void AccelerationTask::Receive(bool blocking)
{
  using barrett::Puck;
  using std::runtime_error;

  typedef barrett::ForceTorqueSensor::AccelParser AccelParser;

  if (state_ != STATE_REQUEST_SENT) {
    return;
  }

  int const ret = Puck::receiveGetPropertyReply<AccelParser>(
    puck_->getBus(), puck_->getId(), prop_id_, accel_, blocking, realtime_);

  if (ret == 0) {
    state_ = STATE_IDLE;
  } else if (ret != 1) {
    throw runtime_error("Failed to receive acceleration.");
  }
}

void AccelerationTask::Write()
{
  // do nothing
}

} // namespace libbarrett_ros
