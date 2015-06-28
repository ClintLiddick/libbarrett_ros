#include <string>
#include <boost/algorithm/string/join.hpp>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <barrett/products/product_manager.h>
#include <barrett/systems/real_time_execution_manager.h>
#include <libbarrett_ros/Bus.h>
#include <libbarrett_ros/BusInfo.h>
#include <libbarrett_ros/Cycle.h>
#include <libbarrett_ros/Schedule.h>
#include <libbarrett_ros/TaskSet.h>
#include <libbarrett_ros/hardware/HandHW.h>
#include <libbarrett_ros/hardware/WamHW.h>
#include <libbarrett_ros/hardware/ForceTorqueSensorHW.h>

namespace libbarrett_ros {

template <class T, size_t N>
static boost::array<T, N> to_array(std::vector<T> const &v)
{
  using boost::format;
  using boost::str;

  if (v.size() != N) {
    throw std::runtime_error(str(
      format("Expected array to be of length %d; got %d.") % N % v.size()));
  }

  boost::array<T, N> output;
  for (size_t i = 0; i < N; ++i) {
    output[i] = v[i];
  }
  return output;
}

static void InitializeSchedule(
  TaskSet const &tasks, ScheduleInfo const &schedule_info, Schedule *schedule)
{
  assert(schedule);

  std::vector<Cycle> cycles(schedule_info.cycles.size());

  for (size_t icycle = 0; icycle < schedule_info.cycles.size(); ++icycle) {
    CycleInfo const &cycle_info = schedule_info.cycles[icycle];
    Cycle &cycle = cycles[icycle];

    BOOST_FOREACH (std::string const &task_name, cycle_info.required_tasks) {
      cycle.AddTask(tasks.GetTask(task_name), true);
    }

    BOOST_FOREACH (std::string const &task_name, cycle_info.optional_tasks) {
      cycle.AddTask(tasks.GetTask(task_name), false);
    }
  }

  *schedule = Schedule(cycles);
}

void InitializeBus(BusInfo const &bus_info, bool is_realtime, Bus *bus)
{
  using boost::format;
  using boost::make_shared;
  using boost::str;
  using barrett::ProductManager;

  // TODO: Can we safely set both of these to false?
  static bool const prompt_on_zeroing = true;
  static bool const wait_for_shift_activate = true;

  // Initialize the ProductManager from configuration files.
  std::string const &config_path = bus_info.configuration_path;
  ROS_INFO_STREAM(
    "Loading configuration file '" << config_path << "' for bus '"
    << bus_info.name << "'.");
  bus->product_manager = make_shared<ProductManager>(config_path.c_str());

  // Stop the libbarrett thread. We'll handle everything oruselves.
  ProductManager &pm = *bus->product_manager;
  pm.getExecutionManager()->stop();

  // TODO: Can we defer some of this to later?
  pm.waitForWam(prompt_on_zeroing);
  pm.wakeAllPucks();

  // TODO: What does this do?
  pm.getSafetyModule()->waitForMode(::barrett::SafetyModule::IDLE);

  bus->hardware.reserve(3);

  if (pm.foundWam7()) {
    ROS_INFO_STREAM("Found a 7-DOF WAM on bus '" << bus_info.name << "'.");
    boost::array<std::string, 7> const wam_joint_names
      = to_array<std::string, 7>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<7> >(
        pm.getWam7(wait_for_shift_activate), is_realtime, &wam_joint_names));
  } else if (pm.foundWam4()) {
    ROS_INFO_STREAM("Found a 4-DOF WAM on bus '" << bus_info.name << "'.");
    boost::array<std::string, 4> const wam_joint_names
      = to_array<std::string, 4>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<4> >(
        pm.getWam4(wait_for_shift_activate), is_realtime, &wam_joint_names));
  } else if (pm.foundWam3()) {
    ROS_INFO_STREAM("Found a 3-DOF WAM on bus '" << bus_info.name << "'.");
    boost::array<std::string, 3> const wam_joint_names
      = to_array<std::string, 3>(bus_info.wam_joint_names);

    bus->hardware.push_back(
      make_shared<WamHW<3> >(
        pm.getWam3(wait_for_shift_activate), is_realtime, &wam_joint_names));
  }

  if (pm.foundHand()) {
    ROS_INFO_STREAM("Found a BarrettHand on bus '" << bus_info.name << "'.");

    barrett::Hand *ft = pm.getHand();
    // TODO: Implement a BarrettHand hardware interface.
    ROS_WARN("The BarrettHand is not yet supported.");
  }

  if (pm.foundForceTorqueSensor()) {
    ROS_INFO_STREAM(
      "Found a force/torque sensor on bus '" << bus_info.name << "'.");

    bus->hardware.push_back(
      make_shared<ForceTorqueSensorHW>(
        pm.getForceTorqueSensor(), is_realtime,
        bus_info.forcetorque_wrench_name,
        bus_info.forcetorque_accel_name,
        bus_info.forcetorque_frame_id));
  }

  // Aggregate all of the tasks on this bus. We'll also build (1) a temporary
  // map that includes the originating hardware interface for debugging
  // purposes and (2) a log message listing the tasks.
  typedef std::map<Task *, BarrettBaseHW *>::iterator Iterator;
  std::map<Task *, BarrettBaseHW *> tasks;
  std::vector<std::string> task_names;
  TaskSet task_set;

  BOOST_FOREACH (boost::shared_ptr<BarrettBaseHW> const &hw, bus->hardware) {
    BOOST_FOREACH (Task *task, hw->tasks()) {
      std::pair<Iterator, bool> const result = tasks.insert(
        std::make_pair(task, hw.get()));
      if (!result.second) {
        throw std::runtime_error(str(
          format("Found duplicate task with name '%s' in hardware"
                 " interfaces '%s' (at %p) and %s (at %p).")
          % task->name()
          % result.first->second->name() % result.first->second
          % hw->name() % hw.get()));
      }

      task_set.AddTask(task);
      task_names.push_back(task->name());
    }
  }

  if (tasks.empty()) {
    ROS_WARN_STREAM(
      "Found zero tasks on bus '" << bus_info.name << "'. Did you specify a"
      " schedule?");
  } else {
    std::sort(task_names.begin(), task_names.end());

    ROS_INFO_STREAM(
      "Found " << tasks.size() << " tasks on bus '" << bus_info.name << "': "
      << boost::algorithm::join(task_names, ", "));
  }

  InitializeSchedule(task_set, bus_info.schedule_info, &bus->schedule);
}

} // namespace libbarrett_ros
