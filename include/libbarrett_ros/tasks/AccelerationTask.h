#ifndef LIBBARRETT_ROS_TASKS_ACCELERATIONTASK_H_
#define LIBBARRETT_ROS_TASKS_ACCELERATIONTASK_H_
#include <barrett/units.h>
#include <libbarrett_ros/Task.h>

namespace barrett {

class Puck;

} // namespace barrett

namespace libbarrett_ros {

class AccelerationTask : public virtual Task {
public:
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  AccelerationTask(barrett::Puck *puck, bool realtime, ca_type *accel);
  virtual ~AccelerationTask();

  virtual std::string const &name() const;

  virtual uint_fast32_t request_bits() const;
  virtual uint_fast32_t receive_bits() const;
  virtual uint_fast32_t write_bits() const;

  virtual void Request();
  virtual void Receive(bool blocking);
  virtual void Write();

private:
  static int const STATE_REQUEST_SENT;
  static int const STATE_IDLE;

  barrett::Puck *puck_; 
  bool realtime_;
  int prop_id_;
  int state_;

  ca_type *accel_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASKS_ACCELERATIONTASK_H_
