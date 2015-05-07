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
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->halt();
  }
}

void BarrettRobotHW::read()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->read();
  }
}

void BarrettRobotHW::write()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->write();
  }
}
