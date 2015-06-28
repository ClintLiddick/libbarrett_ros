#include <cassert>
#include <cmath>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <libbarrett_ros/Schedule.h>
#include <libbarrett_ros/Task.h>

namespace libbarrett_ros {

Schedule::Schedule()
  : state_(STATE_PRECONTROL)
  , phase_(0)
{
}

Schedule::Schedule(std::vector<Cycle> const &cycles)
  : cycles_(cycles)
  , state_(STATE_PRECONTROL)
  , phase_(0)
{
  // Build a list of all tasks. We need this to call Receive() every cycle
  // without introducing duplicates.
  BOOST_FOREACH (Cycle &cycle, cycles_) {
    std::set<Task *> const &required_tasks = cycle.required_tasks();
    std::set<Task *> const &optional_tasks = cycle.optional_tasks();
    tasks_.insert(required_tasks.begin(), required_tasks.end());
    tasks_.insert(optional_tasks.begin(), optional_tasks.end());
  }
}

void Schedule::RunPreControl()
{
  if (cycles_.empty()) {
    return;
  }
  if (state_ != STATE_PRECONTROL) {
    throw std::runtime_error(
      "RunPreControl may only be called once per RunPostControl call.");
  }

  assert(phase_ < cycles_.size());
  Cycle &current_cycle = cycles_[phase_];

  // Make any required requests. The responses will be processed below.
  current_cycle.RequestRequired();

  // There may be pending responses from previous cycles, so we call
  // non-blocking Receive() on all tasks in the Schedule, including the
  // optional tasks in the current cycle. We omit the required tasks in
  // the current cycle so we can do a blocking Receive(); see below.
  std::set<Task *> const &required_tasks = current_cycle.required_tasks();

  BOOST_FOREACH (Task *task, tasks_) {
    if (required_tasks.count(task) == 0) {
      task->Receive(false);
    }
  }

  // Perform a blocking Receive() on any required tasks. We can't proceed to
  // the control calculations until we receive these replies.
  BOOST_FOREACH (Task *task, required_tasks) {
    task->Receive(true);
  }

  // Make any optional requests. The responses will be processed in a future
  // control loop; see above.
  current_cycle.RequestOptional();

  state_ = STATE_POSTCONTROL;
}

void Schedule::RunPostControl()
{
  if (cycles_.empty()) {
    return;
  }
  if (state_ != STATE_POSTCONTROL) {
    throw std::runtime_error(
      "RunPostControl must occur after a RunPreControl call.");
  }

  assert(phase_ < cycles_.size());
  Cycle &current_cycle = cycles_[phase_];

  // TODO: Should WriteOptional be separate? Should it go here?
  current_cycle.WriteRequired();
  current_cycle.WriteOptional();

  phase_ = (phase_ + 1) % cycles_.size();
  state_ = STATE_PRECONTROL;
}

double Schedule::GetUtilization(uint_fast32_t control_period, uint_fast32_t bus_freq) {
  if (control_period == 0) {
    throw std::runtime_error("Control period must be positive.");
  }
  if (bus_freq == 0) {
    throw std::runtime_error("Bus frequency must be positive.");
  }
  if (cycles_.empty()) {
    return 0.;
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
