#ifndef LIBBARRETT_ROS_HARDWARE_HANDHW_H_
#define LIBBARRETT_ROS_HARDWARE_HANDHW_H_
#include <list>
#include <boost/array.hpp>
#include <barrett/products/hand.h>
#include <libbarrett_ros/BarrettBaseHW.h>

// TODO: These should really be constexpr.
#define NUM_FINGERS 3
#define NUM_FINGER_JOINTS 2
#define NUM_SPREAD_JOINTS 2

namespace libbarrett_ros {

class HandHW : public BarrettBaseHW {
public:
  struct JointState {
    double position;
    double velocity;
    double effort;
  };

  struct FingerState {
    JointState inner;
    JointState outer;
  };

  HandHW(barrett::Hand *hand);

  virtual void requestCritical();
  virtual void receiveCritical();

  virtual void requestOther();
  virtual void receiveOther();

  virtual void write();

  virtual void halt();

  virtual void registerHandles(BarrettInterfaces &interfaces);

private:
  barrett::Hand *hand_;
  std::vector<
    barrett::MotorPuck::CombinedPositionParser<
      int>::result_type> position_raw_;

  boost::array<FingerState, NUM_FINGERS> state_;
  JointState spread_state_;

  std::list<size_t> strain_pending_;
  boost::array<int, NUM_FINGERS> strain_;

  bool realtime_;
};

}

#endif // ifndef LIBBARRETT_ROS_HARDWARE_HANDHW_H_
