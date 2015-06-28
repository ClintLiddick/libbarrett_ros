#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <barrett/exception.h>
#include <controller_manager/controller_manager.h>
#include <libbarrett_ros/params.h>
#include <libbarrett_ros/BarrettRobotHW.h>
#include <libbarrett_ros/Bus.h>
#include <libbarrett_ros/BusInfo.h>
#include <libbarrett_ros/ControlLoop.h>

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
  double const publish_freq 
    = get_or_default<double>(root_xmlrpc, "publish_frequency", 100); // 100 Hz

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

  // Start the control loop in a separate (potentially realtime) thread.
  // TODO: This may not work if libbarrett was built with Xenomai support (i.e.
  // -DNON_REALTIME=false), but this controller was run with "realtime" ROS
  // parameter set to "false". ControLoop uses the barrett::PeriodicLoopTimer
  // to run a realtime control thread, but we will be passing "realtime=false"
  // to many libbarrett functions (e.g. Puck::getProperty). This will cause the
  // control thread to drop out of primary mode and produce Xenomai warnings.
  ROS_INFO("Entering control loop.");
  ControlLoop loop(&buses, &cm, control_period, 10);
  loop.Start();

  // Use the main (non-realtime) thread to process ROS messages. This function
  // will exit when the user presses Control+C.
  // TODO: Could the Control+C signal to be caught by the control thread?
  // TODO: Should we use an AsyncSpinner for this?
  ros::Rate rate(publish_freq);

  while (ros::ok() && loop.is_running()) {
    ros::spinOnce();
    rate.sleep();
  }

  // Interrupt the control loop and wait the control thread to exit. This is a
  // no-op if the thread already has stopped.
  loop.Stop();

  if (loop.has_error()) {
    ROS_ERROR_STREAM("Control loop exited with error: "
                     << loop.error_message());
  } else {
    ROS_INFO("Exited control loop.");
  }

  // Idle the hardware to avoid triggering a heartbeat fault.
  ROS_INFO("Idling hardware.");
  robot.halt();

  return 0;
}
