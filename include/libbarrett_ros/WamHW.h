#ifndef LIBBARRETT_ROS_WAMHW_H_
#define LIBBARRETT_ROS_WAMHW_H_
#include <iostream>
#include <string>
#include <boost/array.hpp>
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
    : position_task_(&wam->getLowLevelWam(), realtime,
        &state_position_motor_, &state_position_joint_, &state_position_best_)
    , torque_task_(&wam->getLowLevelWam(), realtime, &command_effort_)
    , llwam_(&wam->getLowLevelWam())
    , state_position_motor_(std::numeric_limits<double>::max())
    , state_position_joint_(std::numeric_limits<double>::max())
    , state_position_best_(std::numeric_limits<double>::max())
    , state_velocity_(0.)
    , state_effort_(0.)
    , command_effort_(0.)
  {
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

  virtual void requestCritical()
  {
    position_task_.Request();
    torque_task_.Request();
  }

  virtual void receiveCritical()
  {
    position_task_.Receive(true);
    torque_task_.Receive(true);
  }

  virtual void requestOther()
  {
  }

  virtual void receiveOther()
  {
  }

  virtual void halt()
  {
    llwam_->getSafetyModule()->setMode(barrett::SafetyModule::IDLE);
    llwam_->getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE, true, 0.05);
  }

  virtual void write()
  {
    position_task_.Write();
    torque_task_.Write();
  }

private:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  barrett::LowLevelWam<DOF> *llwam_;

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

#endif
