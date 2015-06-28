#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <barrett/exception.h>
#include <controller_manager/controller_manager.h>
#include <libbarrett_ros/params.h>
#include <libbarrett_ros/Bus.h>
#include <libbarrett_ros/BusInfo.h>
#include <libbarrett_ros/BarrettRobotHW.h>

int main(int argc, char **argv)
{
  using namespace libbarrett_ros;

  // Installs a new terminate() function that, when no catch clauses handle an
  // exception, prints a stacktrace to stderr before performing the default
  // behavior (terminating the process and generating a core file).
  barrett::installExceptionHandler();

  // Initialize ROS.
  ros::init(argc, argv, "libbarrett_ros_controller");
  ros::NodeHandle nh;

  // TODO: Should we use an AsyncSpinner?
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Read parameters.
  // TODO: Is this the right way to get the private parameter namespace?
  XmlRpc::XmlRpcValue root_xmlrpc;
  ros::param::get("~", root_xmlrpc);

  std::vector<BusInfo> bus_infos;
  try {
    bus_infos = get_or_throw<std::vector<BusInfo> >(root_xmlrpc, "buses");
    ROS_INFO_STREAM("Found" << bus_infos.size() << " communication buses.");
  } catch (std::runtime_error const &e) {
    ROS_FATAL("Failed loading parameters: %s", e.what());
    return 1;
  }

  bool const is_realtime
    = get_or_default<bool>(root_xmlrpc, "realtime", false);
  if (is_realtime) {
    ROS_FATAL("Realtime support is not yet implemented.");
    return 1;
  }

  uint_fast32_t const control_period
    = get_or_default<int>(root_xmlrpc, "control_period", 2000); // 500 Hz
  uint_fast32_t const bus_freq
    = get_or_default<int>(root_xmlrpc, "bus_frequency", 1e6); // 1 Mbps

  // Initialize the communication buses.
  std::vector<Bus> buses(bus_infos.size());
  for (size_t i = 0; i < bus_infos.size(); ++i) {
    BusInfo const &bus_info = bus_infos[i];
    Bus &bus = buses[i];

    ROS_INFO_STREAM("Initializing communication bus " << i << " of "
                    << bus_infos.size() << ".");
    try {
      InitializeBus(bus_info, is_realtime, &bus);
    } catch (std::runtime_error const &e) {
      ROS_FATAL_STREAM("Failed initializating communication bus "
                       << (i + 1) << ": " << e.what());
      return 1;
    }

    // Check bus utilization to, hopefully, avoid a heartbeat fault.
    double const utilization
      = bus.schedule.GetUtilization(control_period, bus_freq);

    if (utilization > bus_info.utilization_error) {
      ROS_FATAL(
        "Communication bus %zu has bus utilization of %.2f%%; exceeds"
        " threshold of %.2f%%.",
        i + 1, 100 * utilization, 100 * bus_info.utilization_error);
      // TODO: Do we need to do any cleanup here?
      return 1;
    } else if (utilization > bus_info.utilization_warn) {
      ROS_WARN(
        "Communication bus %zu has bus utilization of %.2f%%; exceeds"
        " threshold of %.2f%%.",
        i + 1, 100 * utilization, 100 * bus_info.utilization_warn);
    }
  }

  ROS_INFO("Initializing the RobotHW interface.");
  BarrettRobotHW robot;

  BOOST_FOREACH (Bus const &bus, buses) {
    BOOST_FOREACH (boost::shared_ptr<BarrettBaseHW> const &hw, bus.hardware) {
      robot.add(hw);
    }
  }

  ROS_INFO("Activating hardware.");
  robot.initialize();

  ROS_INFO("Initializing the ControllerManager.");
  controller_manager::ControllerManager cm(&robot);

  // Start the control loop. Default to 500 Hz.
  // TODO: ros::Rate and ros::Time are likely not realtime-safe.
  ROS_INFO("Entering control loop.");

  // TODO: This ros::Rate instance ignores the period.
  ros::Duration const period(0, control_period * 1e3);
  ros::Rate r(500);

  while (ros::ok()) {
    BOOST_FOREACH (Bus &bus, buses) {
      bus.schedule.RunPreControl();
    }

    ros::Time const now(::barrett::highResolutionSystemTime());
    cm.update(now, period);

    BOOST_FOREACH (Bus &bus, buses) {
      bus.schedule.RunPostControl();
    }

    r.sleep();
  }

  ROS_INFO("Exited control loop.");

  // Idle the hardware to avoid triggering a heartbeat fault.
  ROS_INFO("Idling hardware.");
  robot.halt();

  return 0;
}
