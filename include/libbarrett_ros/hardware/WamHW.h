#ifndef LIBBARRETT_ROS_HARDWARE_WAMHW_H_
#define LIBBARRETT_ROS_HARDWARE_WAMHW_H_
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <barrett/systems/wam.h>
#include <libbarrett_ros/BarrettBaseHW.h>
#include <libbarrett_ros/tasks/WAMPositionTask.h>
#include <libbarrett_ros/tasks/WAMTorqueTask.h>

namespace libbarrett_ros {

template <size_t DOF>
class WamHW : public BarrettBaseHW {
public:
  WamHW(barrett::systems::Wam<DOF> *wam, bool realtime,
        boost::array<std::string, DOF> const *joint_names = NULL)
    : tasks_(2)
    , position_task_(&wam->getLowLevelWam(), realtime,
        &state_position_motor_, &state_position_joint_, &state_position_best_)
    , torque_task_(&wam->getLowLevelWam(), realtime, &command_effort_)
    , llwam_(&wam->getLowLevelWam())
    , state_position_motor_(std::numeric_limits<double>::quiet_NaN())
    , state_position_joint_(std::numeric_limits<double>::quiet_NaN())
    , state_position_best_(std::numeric_limits<double>::quiet_NaN())
    , state_velocity_(0.)
    , state_effort_(0.)
    , command_effort_(0.)
  {
    tasks_[0] = &position_task_;
    tasks_[1] = &torque_task_;

    if (joint_names) {
      joint_names_ = *joint_names;
    } else {
      for (size_t i = 0; i < DOF; ++i) {
        joint_names_[i] = boost::str(boost::format("wam_joint_%d") % (i + 1));
      }
    }
  }

  virtual ~WamHW()
  {
  }

  virtual std::vector<Task *> const &tasks() const
  {
    return tasks_;
  }

  virtual std::string const &name() const
  {
    using boost::format;
    using boost::str;

    static std::string const name = str(format("WAM%d") % DOF);
    return name;
  }

  virtual void registerHandles(BarrettInterfaces &interfaces)
  {
    for (size_t i = 0; i < DOF; ++i) {
      hardware_interface::JointStateHandle jointstate_handle(
        joint_names_[i],
        &state_position_best_[i], &state_velocity_[i], &state_effort_[i]
      );
      interfaces.jointstate_interface.registerHandle(jointstate_handle);

      interfaces.jointeffort_interface.registerHandle(
        ::hardware_interface::JointHandle(
          jointstate_handle, &command_effort_[i]
        )
      );
    }
  }

  virtual void halt()
  {
    llwam_->getSafetyModule()->setMode(barrett::SafetyModule::IDLE);
    llwam_->getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE, true, 0.05);
  }

private:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  barrett::LowLevelWam<DOF> *llwam_;

  std::vector<Task *> tasks_;
  WAMPositionTask<DOF> position_task_;
  WAMTorqueTask<DOF> torque_task_;

  boost::array<std::string, DOF> joint_names_;

  barrett::math::Matrix<DOF, 2> raw_position_;
  jp_type state_position_motor_;
  jp_type state_position_joint_;
  jp_type state_position_best_;

  jv_type state_velocity_;
  jt_type state_effort_;
  jt_type command_effort_;

  DISALLOW_COPY_AND_ASSIGN(WamHW);
};

}

#endif // ifndef LIBBARRETT_ROS_HARDWARE_WAMHW_H_
