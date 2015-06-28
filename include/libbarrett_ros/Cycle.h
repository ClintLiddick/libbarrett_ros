#ifndef LIBBARRETT_ROS_CYCLE_H_
#define LIBBARRETT_ROS_CYCLE_H_
#include <stdint.h>
#include <set>

namespace libbarrett_ros {

class Task;

class Cycle {
public:
  uint_fast32_t bits() const;

  void AddTask(Task *task, bool is_required);

  std::set<Task *> const &required_tasks() const;
  std::set<Task *> const &optional_tasks() const;

  void RequestRequired();
  void ReceiveRequired();
  void WriteRequired();

  void RequestOptional();
  void ReceiveOptional();
  void WriteOptional();
 
private:
  std::set<Task *> required_tasks_;
  std::set<Task *> optional_tasks_; 
};

} // namespace libbarrett_ros

#endif
