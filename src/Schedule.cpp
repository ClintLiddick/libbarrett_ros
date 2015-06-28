#include <cassert>
#include <cmath>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <libbarrett_ros/Schedule.h>

namespace libbarrett_ros {

Schedule::Schedule(std::vector<Cycle> const &cycles)
  : cycles_(cycles)
  , state_(STATE_PRECONTROL)
  , phase_(0)
{
  if (cycles.empty()) {
    throw std::runtime_error("At least one cycle must be specified.");
  }
}

void Schedule::RunPreControl()
{
  if (state_ != STATE_PRECONTROL) {
    throw std::runtime_error(
      "RunPreControl may only be called once per RunPostControl call.");
  }

  assert(phase_ < cycles_.size());
  Cycle &current_cycle = cycles_[phase_];

  current_cycle.RequestRequired();
  current_cycle.ReceiveOptional();
  current_cycle.ReceiveRequired();
  current_cycle.RequestOptional();

  state_ = STATE_POSTCONTROL;
}

void Schedule::RunPostControl()
{
  if (state_ != STATE_POSTCONTROL) {
    throw std::runtime_error(
      "RunPostControl must occur after a RunPreControl call.");
  }

  assert(phase_ < cycles_.size());
  Cycle &current_cycle = cycles_[phase_];

  // TODO: Should WriteOptional go here?
  current_cycle.WriteRequired();
  current_cycle.WriteOptional();

  phase_ = (phase_ + 1) % cycles_.size();
  state_ = STATE_PRECONTROL;
}

double Schedule::GetUtilization(uint_fast32_t control_period,
                                uint_fast32_t bus_freq)
{
  if (control_period == 0) {
    throw std::runtime_error("Control period must be positive.");
  }
  if (bus_freq == 0) {
    throw std::runtime_error("Bus frequency must be positive.");
  }

  // Compute the total time (in us) per execution.
  uint_fast32_t const allowed_us = control_period * cycles_.size();
  
  // Compute the total number of bits transmitted per execution.
  uint_fast32_t total_bits = 0;

  BOOST_FOREACH (Cycle const &cycle, cycles_) {
    total_bits += cycle.bits();
  }

  // Compute the time (in us) that the bus is active.
  double active_us = std::ceil(static_cast<double>(total_bits) / bus_freq);

  // Compute the utilization ratio.
  return active_us / allowed_us;
}

} // namespace libbarrett_ros
