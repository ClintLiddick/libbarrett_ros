#ifndef LIBBARRETT_ROS_BUS_H_
#define LIBBARRETT_ROS_BUS_H_
#include <vector>
#include <boost/shared_ptr.hpp>
#include <libbarrett_ros/BarrettBaseHW.h>
#include <libbarrett_ros/Schedule.h>

namespace barrett {

class ProductManager;

} // namespace barrett

namespace libbarrett_ros {

class BusInfo;

struct Bus {
  boost::shared_ptr<barrett::ProductManager> product_manager;
  std::vector<boost::shared_ptr<BarrettBaseHW> > hardware;
  Schedule schedule;
};

void InitializeBus(BusInfo const &bus_info, bool is_realtime, Bus *bus);

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_BUS_H_
