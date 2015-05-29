#ifndef LIBBARRETT_ROS_FORCETORQUESENSORHW_H_
#define LIBBARRETT_ROS_FORCETORQUESENSORHW_H_
#include <string>
#include <barrett/products/force_torque_sensor.h>
#include <libbarrett_ros/BarrettBaseHW.h>

namespace libbarrett_ros {

class ForceTorqueSensorHW : public BarrettBaseHW {
public:
  ForceTorqueSensorHW(
    barrett::ForceTorqueSensor *forcetorque_sensor,
    std::string const &forcetorque_name, std::string const &accelerometer_name,
    std::string const &frame_id);
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

  static int const ACCEL_REQUEST_SENT;
  static int const ACCEL_IDLE;

  static int const FORCETORQUE_REQUEST_SENT;
  static int const FORCETORQUE_RECEIVED_FORCE;
  static int const FORCETORQUE_RECEIVED_TORQUE;
  static int const FORCETORQUE_IDLE;

  std::string forcetorque_name_;
  std::string accelerometer_name_;
  std::string frame_id_;
  cf_type force_;
  ct_type torque_;
  ca_type accel_;

  bool realtime_;
  int accel_state_;
  int ft_state_;

  barrett::ForceTorqueSensor *sensor_;

  void requestForceTorque();
  void receiveForceTorque(bool blocking);

  void requestAccel();
  void receiveAccel(bool blocking);

};

}

#endif
