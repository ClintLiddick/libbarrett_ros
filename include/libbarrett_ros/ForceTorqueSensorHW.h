#ifndef LIBBARRETT_ROS_FORCETORQUESENSORHW_H_
#define LIBBARRETT_ROS_FORCETORQUESENSORHW_H_
#include <string>
#include <barrett/products/force_torque_sensor.h>
#include <libbarrett_ros/BarrettBaseHW.h>

namespace libbarrett_ros {

class ForceTorqueSensorHW : public BarrettBaseHW {
public:
  ForceTorqueSensorHW(
      ::barrett::ForceTorqueSensor *forcetorque_sensor,
      std::string const &name, std::string const &frame_id);
  virtual ~ForceTorqueSensorHW();

  virtual void registerHandles(BarrettInterfaces &interfaces);

  virtual void requestCritical();
  virtual void receiveCritical();

  virtual void requestOther();
  virtual void receiveOther();

  virtual void write();

  virtual void halt();

private:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  static int const STATE_REQUEST_SENT;
  static int const STATE_RECEIVED_FORCE;
  static int const STATE_RECEIVED_TORQUE;
  static int const STATE_IDLE;

  std::string name_;
  std::string frame_id_;
  cf_type force_;
  ct_type torque_;
  ca_type accel_;
  int state_;

  ::barrett::ForceTorqueSensor *sensor_;
};

}

#endif
