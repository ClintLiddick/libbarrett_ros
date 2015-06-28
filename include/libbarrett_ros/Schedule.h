#ifndef LIBBARRETT_ROS_SCHEDULE_H_
#define LIBBARRETT_ROS_SCHEDULE_H_
#include <cstddef>
#include <vector>
#include <set>
#include <libbarrett_ros/Cycle.h>

namespace libbarrett_ros {

class Schedule {
public:
  Schedule();
  Schedule(std::vector<Cycle> const &cycles);

  void RunPreControl();
  void RunPostControl();

  double GetUtilization(uint_fast32_t control_period, uint_fast32_t bus_freq);

private:
  enum State {
    STATE_PRECONTROL,
    STATE_POSTCONTROL
  };

  std::vector<Cycle> cycles_;
  std::set<Task *> tasks_;
  State state_;
  size_t phase_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_SCHEDULE_H_
