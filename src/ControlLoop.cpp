#include <cassert>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <libbarrett_ros/Bus.h>
#include <libbarrett_ros/ControlLoop.h>

namespace libbarrett_ros {

ControlLoop::ControlLoop(
    std::vector<Bus> *buses,
    controller_manager::ControllerManager *controller_manager,
    double period, int thread_priority)
  : buses_(buses)
  , controller_manager_(controller_manager)
  , loop_timer_(period, thread_priority)
  , period_(period)
{
  assert(buses);
  assert(controller_manager);

  if (!running_.is_lock_free()) {
    ROS_WARN("boost::atomic<bool> is not lock free. This may impact realtime"
             " safety.");
  }
  // The value of is_lock_free() is guaranteed to be consistent between
  // instances of the same boost::atomic<T> class, so we don't need to
  // check error_ separately.

  running_.store(false);
  error_.store(false);
}

ControlLoop::~ControlLoop()
{
  // TODO: Exit the thread if it's still running.
}

bool ControlLoop::is_running() const
{
  return running_.load(boost::memory_order_acquire);
}

bool ControlLoop::has_error() const
{
  return error_.load(boost::memory_order_acquire);
}

std::string ControlLoop::error_message() const
{
  // We use memory_order_release to guarantee that error_message_ is set before
  // has_error_ changes, so accessing error_message_ is safe.
  if (has_error()) {
    return error_message_;
  } else {
    return "";
  }
}

void ControlLoop::Start()
{
  thread_ = boost::thread(&ControlLoop::Loop, this);
  running_.store(true);
  error_ = false;
  error_message_ = "";
}

void ControlLoop::Stop()
{
  thread_.interrupt();
  thread_.join();
}

void ControlLoop::Loop()
{
  ros::Duration const period_ros(period_);

  try {
    for (;;) {
      boost::this_thread::interruption_point();

      BOOST_FOREACH (Bus &bus, *buses_) {
        bus.schedule.RunPreControl();
      }

      ros::Time const now(barrett::highResolutionSystemTime());
      controller_manager_->update(now, period_ros);

      BOOST_FOREACH (Bus &bus, *buses_) {
        bus.schedule.RunPostControl();
      }

      loop_timer_.wait();
    }
  } catch (boost::thread_interrupted const &e) {
    // Interruption requested, probably by stop(). Do nothing.
  } catch (std::runtime_error const &e) {
    error_message_ = e.what();
    error_.store(true, boost::memory_order_release);
  }

  running_.store(false, boost::memory_order_release);
}

} // namespace libbarrett_ros
