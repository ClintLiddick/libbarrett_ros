#ifndef LIBBARRETT_ROS_TASKS_WAMPOSITION_H_
#define LIBBARRETT_ROS_TASKS_WAMPOSITION_H_
#include <libbarrett_ros/Task.h>
#include <barrett/units.h>
#include <barrett/products/low_level_wam.h>

namespace libbarrett_ros {

template <size_t DOF>
class WAMPositionTask : public virtual Task {
public:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  WAMPositionTask(barrett::LowLevelWam<DOF> *llwam, bool realtime,
    jp_type *position_motor, jp_type *position_joint, jp_type *position_best);

  virtual ~WAMPositionTask();

  virtual std::string const &name() const;

  virtual uint_fast32_t request_bits() const;
  virtual uint_fast32_t receive_bits() const;
  virtual uint_fast32_t write_bits() const;

  virtual void Request();
  virtual void Receive(bool blocking);
  virtual void Write();

private:
  barrett::LowLevelWam<DOF> *llwam_;

  bool realtime_;
  int prop_id_;

  barrett::math::Matrix<DOF, 2> raw_position_;
  jp_type *position_motor_;
  jp_type *position_joint_;
  jp_type *position_best_;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASKS_WAMPOSITION_H_
