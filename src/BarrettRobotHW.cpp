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

void BarrettRobotHW::requestCritical()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->requestCritical();
  }
}

void BarrettRobotHW::receiveCritical()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->receiveCritical();
  }
}

void BarrettRobotHW::requestOther()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->requestOther();
  }
}

void BarrettRobotHW::receiveOther()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->receiveOther();
  }
}

void BarrettRobotHW::write()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->write();
  }
}

void BarrettRobotHW::halt()
{
  for (size_t i = 0; i < hardware_.size(); ++i) {
    hardware_[i]->halt();
  }
}
