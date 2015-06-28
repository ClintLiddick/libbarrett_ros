#ifndef LIBBARRETT_ROS_CONTROLLOOP_H_
#define LIBBARRETT_ROS_CONTROLLOOP_H_
#include <vector>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <barrett/os.h>

namespace controller_manager {

class ControllerManager;

} // namespace controller_manager

namespace libbarrett_ros {

class Bus;

class ControlLoop {
public:
  ControlLoop(
    std::vector<Bus> *buses,
    controller_manager::ControllerManager *controller_manager,
    double period, int thread_priority);

  ~ControlLoop();

  bool is_running() const;
  bool has_error() const;
  std::string error_message() const;

  void Start();
  void Stop();

private:
  void Loop();

  std::vector<Bus> *buses_;
  controller_manager::ControllerManager *controller_manager_;
  barrett::PeriodicLoopTimer loop_timer_;
  double period_;

  boost::atomic<bool> running_;
  boost::atomic<bool> error_;
  std::string error_message_;
  mutable boost::thread thread_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_BUS_H_
