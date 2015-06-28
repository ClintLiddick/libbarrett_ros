#ifndef LIBBARRETT_ROS_BUSINFO_H_
#define LIBBARRETT_ROS_BUSINFO_H_
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/filesystem/operations.hpp>
#include <libbarrett_ros/params.h>
#include <libbarrett_ros/HandHW.h> // for NUM_*_JOINTS

namespace libbarrett_ros {

struct CycleInfoEntry {
  std::string task_name;
  bool required;
};

struct CycleInfo {
  std::vector<std::string> required_tasks;
  std::vector<std::string> optional_tasks;
};

struct ScheduleInfo {
  std::vector<CycleInfo> cycles;
};

struct BusInfo {
  std::string name;
  std::string configuration_path;

  std::vector<std::string> wam_joint_names;

  boost::array<boost::array<std::string, NUM_FINGER_JOINTS>,
    NUM_FINGERS> hand_finger_joint_names;
  boost::array<std::string, NUM_SPREAD_JOINTS> hand_spread_joint_names;

  std::string forcetorque_wrench_name;
  std::string forcetorque_accel_name;
  std::string forcetorque_frame_id;

  double utilization_warn;
  double utilization_error;

  ScheduleInfo schedule_info;
};

/*
 * XmlRpcValue conversions
 */
template <>
struct get_value_impl<CycleInfoEntry> {
  typedef CycleInfoEntry element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpc::XmlRpcValue const &xmlrpc);
};

template <>
struct get_value_impl<CycleInfo> {
  typedef CycleInfo element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpc::XmlRpcValue const &xmlrpc);
};

template <>
struct get_value_impl<ScheduleInfo> {
  typedef ScheduleInfo element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpc::XmlRpcValue const &xmlrpc);
};

template <>
struct get_value_impl<BusInfo> {
  typedef BusInfo element_type;
  typedef element_type return_type;
  
  static return_type call(XmlRpc::XmlRpcValue const &xmlrpc);
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_BUSINFO_H_
