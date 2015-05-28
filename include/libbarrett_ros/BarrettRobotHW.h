#ifndef LIBBARRETT_ROS_BARRETTROBOTHW_H_
#define LIBBARRETT_ROS_BARRETTROBOTHW_H_
#include <vector>
#include <boost/shared_ptr.hpp>
#include <hardware_interface/robot_hw.h>
#include <libbarrett_ros/BarrettBaseHW.h>

namespace libbarrett_ros {

class BarrettRobotHW : public ::hardware_interface::RobotHW {
public:
  BarrettRobotHW();
  virtual ~BarrettRobotHW();

  void initialize();
  void add(boost::shared_ptr<BarrettBaseHW> const &hardware);

  void requestCritical();
  void receiveCritical();

  void requestOther();
  void receiveOther();

  void write();

  void halt();

private:
  BarrettInterfaces interfaces_;
  std::vector<boost::shared_ptr<BarrettBaseHW> > hardware_;
};

}

#endif
