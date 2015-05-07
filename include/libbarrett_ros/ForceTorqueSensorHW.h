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

  virtual void halt();
  virtual void read();
  virtual void write();

private:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  std::string name_;
  std::string frame_id_;
  cf_type force_;
  cf_type torque_;
	ca_type accel_;

  ::barrett::ForceTorqueSensor *sensor_;
};

}

#endif
