#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <libbarrett_ros/Cycle.h>
#include <libbarrett_ros/Task.h>

namespace libbarrett_ros {

uint_fast32_t Cycle::bits() const
{
  uint_fast32_t sum = 0;

  BOOST_FOREACH (Task *task, required_tasks_) {
    sum += task->request_bits() + task->receive_bits() + task->write_bits();
  }

  BOOST_FOREACH (Task *task, optional_tasks_) {
    sum += task->request_bits() + task->receive_bits() + task->write_bits();
  }

  return sum; 
}

void Cycle::AddTask(Task *task, bool is_required)
{
  typedef std::set<Task *>::iterator Iterator;

  using boost::format;
  using boost::str;

  std::set<Task *> *task_set = &required_tasks_;
  std::set<Task *> *other_set = &optional_tasks_;
  if (!is_required) {
    std::swap(task_set, other_set);
  }

  Iterator const tasks_it = task_set->find(task);
  Iterator const other_it = other_set->find(task);

  if (tasks_it == task_set->end() && other_it == task_set->end()) {
    task_set->insert(task);
  } else {
    throw std::runtime_error(str(
      format("Attempted to add duplicate task '%s'.") % task->name()));
  }
}

void Cycle::RequestRequired()
{
  BOOST_FOREACH (Task *task, required_tasks_) {
    task->Request();
  }
}

void Cycle::ReceiveRequired()
{
  BOOST_FOREACH (Task *task, required_tasks_) {
    task->Receive(true); // blocking
  }
}

void Cycle::WriteRequired()
{
  BOOST_FOREACH (Task *task, required_tasks_) {
    task->Write();
  }
}

void Cycle::RequestOptional()
{
  BOOST_FOREACH (Task *task, optional_tasks_) {
    task->Request();
  }
}

void Cycle::ReceiveOptional()
{
  BOOST_FOREACH (Task *task, optional_tasks_) {
    task->Receive(false); // non-blocking
  }
}

void Cycle::WriteOptional()
{
  BOOST_FOREACH (Task *task, optional_tasks_) {
    task->Write();
  }
}

} // namespace libbarrett_ros
