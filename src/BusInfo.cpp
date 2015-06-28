#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <libbarrett_ros/BusInfo.h>

using XmlRpc::XmlRpcValue;

namespace libbarrett_ros {

CycleInfoEntry get_value_impl<CycleInfoEntry>::call(XmlRpcValue const &xmlrpc)
{
  CycleInfoEntry entry;
  entry.task_name = get_or_throw<std::string>(xmlrpc, "name");
  entry.required = get_or_default<bool>(xmlrpc, "required", false);
  return entry;
}

CycleInfo get_value_impl<CycleInfo>::call(XmlRpcValue const &xmlrpc)
{
  std::vector<CycleInfoEntry> const entries 
    = get_value_impl<std::vector<CycleInfoEntry> >::call(xmlrpc);
  CycleInfo cycle_info;

  BOOST_FOREACH (CycleInfoEntry const &entry, entries) {
    if (entry.required) {
      cycle_info.required_tasks.push_back(entry.task_name);
    } else {
      cycle_info.optional_tasks.push_back(entry.task_name);
    }
  }

  return cycle_info;
}

ScheduleInfo get_value_impl<ScheduleInfo>::call(XmlRpcValue const &xmlrpc)
{
  ScheduleInfo schedule_info;
  schedule_info.cycles = get_value_impl<std::vector<CycleInfo> >::call(xmlrpc);
  return schedule_info;
}

BusInfo get_value_impl<BusInfo>::call(XmlRpcValue const &xmlrpc)
{
  using boost::assign::list_of;
  using boost::filesystem::is_directory;
  using boost::format;
  using boost::str;

  static std::vector<std::string> const default_wam_joint_names
    = list_of("wam_joint1")("wam_joint2")("wam_joint3")("wam_joint4")
             ("wam_joint5")("wam_joint6")("wam_joint7");
  static boost::array<boost::array<std::string, NUM_FINGER_JOINTS>,
                      NUM_FINGERS> const default_hand_finger_joint_names
    = list_of(list_of("finger0_1")("finger0_2"))
             (list_of("finger1_1")("finger1_2"))
             (list_of("finger2_1")("finger2_2"));
  static boost::array<std::string, NUM_SPREAD_JOINTS> const
    default_hand_spread_joint_names
      = list_of("finger0_0")("finger1_0");

  BusInfo bus_info;

  bus_info.configuration_path
    = get_or_throw<std::string>(xmlrpc, "configuration_path");
  if (is_directory(bus_info.configuration_path)) {
    throw std::runtime_error(str(
      format("Configuration path '%s' is a directory, but should be a file."
             " Did you forget to include 'default.conf'?")
        % bus_info.configuration_path));
  }

  bus_info.wam_joint_names
    = get_or_default<std::vector<std::string> >(
        xmlrpc, "wam_joint_names", default_wam_joint_names);
  bus_info.hand_finger_joint_names
    = get_or_default<boost::array<
        boost::array<std::string, NUM_FINGER_JOINTS>, NUM_FINGERS> >(
          xmlrpc, "hand_finger_joint_names", default_hand_finger_joint_names);
  bus_info.hand_spread_joint_names
    = get_or_default<boost::array<std::string, NUM_SPREAD_JOINTS> >(
        xmlrpc, "hand_spread_joint_names", default_hand_spread_joint_names);

  bus_info.forcetorque_wrench_name = get_or_default<std::string>(
    xmlrpc, "forcetorque_wrench_name", "ft_wrench");
  bus_info.forcetorque_accel_name = get_or_default<std::string>(
    xmlrpc, "forcetorque_accel_name", "ft_accel");
  bus_info.forcetorque_frame_id = get_or_default<std::string>(
    xmlrpc, "forcetorque_frame_id", "wam7");

  bus_info.utilization_warn
    = get_or_default<double>(xmlrpc, "utilization_threshold_warning", 0.9);
  bus_info.utilization_error
    = get_or_default<double>(xmlrpc, "utilization_threshold_error", 1.0);
}

} // namespace libbarrett_ros
