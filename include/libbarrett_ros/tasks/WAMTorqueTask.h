#ifndef LIBBARRETT_ROS_TASKS_WAMTORQUE_H_
#define LIBBARRETT_ROS_TASKS_WAMTORQUE_H_
#include <libbarrett_ros/Task.h>
#include <barrett/units.h>
#include <barrett/products/low_level_wam.h>

namespace libbarrett_ros {

template <size_t DOF>
class WAMTorqueTask : public virtual Task {
public:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  WAMTorqueTask(barrett::LowLevelWam<DOF> *llwam, bool realtime,
                jt_type *command_effort);

  virtual ~WAMTorqueTask();

  virtual std::string const &name() const;

  virtual uint_fast32_t request_bits() const;
  virtual uint_fast32_t receive_bits() const;
  virtual uint_fast32_t write_bits() const;

  virtual void Request();
  virtual void Receive(bool blocking);
  virtual void Write();

private:
  barrett::LowLevelWam<DOF> *llwam_;

  jt_type *command_effort_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASKS_WAMTORQUE_H_
