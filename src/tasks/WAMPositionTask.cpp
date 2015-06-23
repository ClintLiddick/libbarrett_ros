#include <libbarrett_ros/tasks/WAMPositionTask.h>

namespace libbarrett_ros {

template <size_t DOF>
WAMPositionTask<DOF>::WAMPositionTask(
    barrett::LowLevelWam<DOF> *llwam, bool realtime,
    jp_type *position_motor, jp_type *position_joint, jp_type *position_best)
  : llwam_(llwam)
  , realtime_(realtime)
  , raw_position_(0.)
  , position_motor_(position_motor)
  , position_joint_(position_joint)
  , position_best_(position_best)
{
  using barrett::Puck;

  prop_id_ = llwam_->getPuckGroup().getPropertyId(Puck::P);
}

template <size_t DOF>
WAMPositionTask<DOF>::~WAMPositionTask()
{
}

template <size_t DOF>
void WAMPositionTask<DOF>::Request()
{
  llwam_->getPuckGroup().sendGetPropertyRequest(prop_id_);
}

template <size_t DOF>
void WAMPositionTask<DOF>::Receive(bool blocking)
{
  using ::barrett::Puck;
  using ::barrett::PuckGroup;
  using ::barrett::MotorPuck;

  typedef MotorPuck::CombinedPositionParser<double> CombinedPositionParser;

  // TODO: The receiveGetPropertyReply method on PuckGroup does not support
  // non-blocking operation. We'll have to implement this ourselves: see
  // ForceTorqueTask for an example.
  if (!blocking) {
    throw std::runtime_error(
      "WAMPositionTask::Receive does not support non-blocking operation.");
  }

  BOOST_STATIC_ASSERT(
    sizeof(CombinedPositionParser::result_type) == 2 * sizeof(double));

  // Receive the puck positions.
  PuckGroup const &group = llwam_->getPuckGroup();

  group.receiveGetPropertyReply<CombinedPositionParser>(
    prop_id_,
    reinterpret_cast<CombinedPositionParser::result_type *>(
      raw_position_.data()),
    realtime_
  );

  // Converts motor encoder to joint positions.
  *position_motor_ = llwam_->getPuckToJointPositionTransform()
                     * raw_position_.col(0);

  // Read joint encoders, if they're available.
  for (size_t i = 0; i < DOF; ++i) {
    if (raw_position_(i, 1) == std::numeric_limits<double>::max()) {
      (*position_joint_)[i] = std::numeric_limits<double>::max();
      (*position_best_)[i] = (*position_motor_)[i];
    } else {
      (*position_joint_)[i]
        = llwam_->getJointEncoderToJointPositionTransform()[i]
          * raw_position_(i, 1);

      if (llwam_->usingJointEncoder(i)) {
        (*position_best_)[i] = (*position_joint_)[i];
      } else {
        (*position_best_)[i] = (*position_motor_)[i];
      }
    }
  }
}

template <size_t DOF>
void WAMPositionTask<DOF>::Write()
{
  // do nothing
}

// Explicitly instantiate the versions of WAMPositionTask that we need.
template class WAMPositionTask<3>;
template class WAMPositionTask<4>;
template class WAMPositionTask<7>;

} // namespace libbarrett_ros
