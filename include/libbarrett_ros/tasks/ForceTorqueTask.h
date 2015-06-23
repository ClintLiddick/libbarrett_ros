#ifndef LIBBARRETT_ROS_TASKS_FORCETORQUETASK_H_
#define LIBBARRETT_ROS_TASKS_FORCETORQUETASK_H_
#include <libbarrett_ros/Task.h>

namespace barrett {

class Puck;

} // namespace barrett

namespace libbarrett_ros {

class ForceTorqueTask : public virtual Task {
public:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  ForceTorqueTask(barrett::Puck *puck, bool realtime,
                  cf_type *force, ct_type *torque);
  virtual ~ForceTorqueTask();

  virtual void Request();
  virtual void Receive(bool blocking);
  virtual void Write();

private:
  static int const STATE_REQUEST_SENT;
  static int const STATE_RECEIVED_FORCE;
  static int const STATE_RECEIVED_TORQUE;
  static int const STATE_IDLE;

  barrett::Puck *puck_; 
  bool realtime_;
  int prop_id_;
  int state_;

  cf_type *force_;
  ct_type *torque_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASKS_FORCETORQUETASK_H_
