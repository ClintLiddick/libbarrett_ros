#include <cassert>
#include <boost/format.hpp>
#include <libbarrett_ros/Task.h>
#include <libbarrett_ros/TaskSet.h>

using boost::format;
using boost::str;

namespace libbarrett_ros {

void TaskSet::AddTask(Task *task)
{
  typedef std::map<std::string, Task *>::iterator Iterator;

  assert(task);

  std::pair<Iterator, bool> const result = tasks_.insert(
    std::make_pair(task->name(), task));
  if (!result.second) {
    throw std::runtime_error(str(
      format("Attempted to add duplicate task '%s'.") % task->name()));
  }
}

Task *TaskSet::GetTask(std::string const &name) const
{
  typedef std::map<std::string, Task *>::const_iterator Iterator;

  Iterator const it = tasks_.find(name);
  if (it != tasks_.end()) {
    return it->second;
  } else {
    throw std::runtime_error(str(
      format("There is no task named '%s'.") % name));
  }
}

} // namespace libbarrett_ros
