#ifndef LIBBARRETT_ROS_HARDWARE_FORCETORQUESENSORHW_H_
#define LIBBARRETT_ROS_HARDWARE_FORCETORQUESENSORHW_H_
#include <string>
#include <barrett/products/force_torque_sensor.h>
#include <libbarrett_ros/BarrettBaseHW.h>
#include <libbarrett_ros/tasks/AccelerationTask.h>
#include <libbarrett_ros/tasks/ForceTorqueTask.h>

namespace libbarrett_ros {

class ForceTorqueSensorHW : public BarrettBaseHW {
public:
  ForceTorqueSensorHW(
    barrett::ForceTorqueSensor *forcetorque_sensor, bool realtime,
    std::string const &forcetorque_name, std::string const &accelerometer_name,
    std::string const &frame_id);
  virtual ~ForceTorqueSensorHW();

  virtual std::vector<Task *> const &tasks() const;

  virtual void registerHandles(BarrettInterfaces &interfaces);

  virtual void halt();

private:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  std::vector<Task *> tasks_;
  ForceTorqueTask ft_task_;
  AccelerationTask accel_task_;

  std::string forcetorque_name_;
  std::string accelerometer_name_;
  std::string frame_id_;
  cf_type force_;
  ct_type torque_;
  ca_type accel_;

  bool realtime_;

  barrett::ForceTorqueSensor *sensor_;
};

}

#endif // ifndef LIBBARRETT_ROS_HARDWARE_FORCETORQUESENSORHW_H_
