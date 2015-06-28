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
std::string const &WAMTorqueTask<DOF>::name() const
{
  static std::string const name = "wam_torque";
  return name;
}

template <size_t DOF>
uint_fast32_t WAMTorqueTask<DOF>::request_bits() const
{
  return 0; // no request
}

template <size_t DOF>
uint_fast32_t WAMTorqueTask<DOF>::receive_bits() const
{
  return 0; // no request
}

template <size_t DOF>
uint_fast32_t WAMTorqueTask<DOF>::write_bits() const
{
  if (DOF <= 4) {
    // One packed torque message of four 14-bit values:
    // 47 bits header + 8 bytes payload
    // TODO: Barrett quotes 125 us for this.
    return 47 + 8 * 8;
  } else {
    // Two packed torque messages (see above).
    // TODO: Barrett quotes 250 us for this.
    return 2 * (47 + 8 * 8);
  }
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
