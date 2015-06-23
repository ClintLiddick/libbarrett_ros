#include <libbarrett_ros/tasks/WAMTorqueTask.h>

namespace libbarrett_ros {

template <size_t DOF>
WAMTorqueTask<DOF>::WAMTorqueTask(barrett::LowLevelWam<DOF> *llwam,
                                  bool realtime, jt_type *command_effort)
  : command_effort_(command_effort)
{
}

template <size_t DOF>
WAMTorqueTask<DOF>::~WAMTorqueTask()
{
}

template <size_t DOF>
void WAMTorqueTask<DOF>::Request()
{
  // do nothing
}

template <size_t DOF>
void WAMTorqueTask<DOF>::Receive(bool blocking)
{
  // do nothing
}

template <size_t DOF>
void WAMTorqueTask<DOF>::Write()
{
  // TODO: What does this do? I'd prefer to call the low-level puck command.
  llwam_->setTorques(*command_effort_);
}

// Explicitly instantiate the versions of WAMTorqueTask that we need.
template class WAMTorqueTask<3>;
template class WAMTorqueTask<4>;
template class WAMTorqueTask<7>;

} // namespace libbarrett_ros
