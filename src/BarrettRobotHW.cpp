#include <boost/foreach.hpp>
#include <libbarrett_ros/BarrettRobotHW.h>

using libbarrett_ros::BarrettBaseHW;
using libbarrett_ros::BarrettRobotHW;

BarrettRobotHW::BarrettRobotHW()
{
}

BarrettRobotHW::~BarrettRobotHW()
{
}

void BarrettRobotHW::add(boost::shared_ptr<BarrettBaseHW> const &hardware)
{
  hardware_.push_back(hardware);
  hardware->registerHandles(interfaces_);
}

void BarrettRobotHW::initialize()
{
  interfaces_.registerAll(*this);
}

void BarrettRobotHW::halt()
{
  BOOST_FOREACH (boost::shared_ptr<BarrettBaseHW> const &hw, hardware_) {
    hw->halt();
  }
}
