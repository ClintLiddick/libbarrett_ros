#ifndef LIBBARRETT_ROS_TASKSET_H_
#define LIBBARRETT_ROS_TASKSET_H_
#include <string>
#include <map>

namespace libbarrett_ros {

class Task;

class TaskSet {
public:
  void AddTask(Task *task);
  Task *GetTask(std::string const &name) const;

private:
  std::map<std::string, Task *> tasks_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASKSET_H_
