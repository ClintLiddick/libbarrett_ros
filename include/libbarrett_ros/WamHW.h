#ifndef LIBBARRETT_ROS_WAMHW_H_
#define LIBBARRETT_ROS_WAMHW_H_
#include <string>
#include <boost/array.hpp>
#include <barrett/systems/wam.h>
#include <libbarrett_ros/BarrettBaseHW.h>

namespace libbarrett_ros {

template <size_t DOF>
class WamHW : public BarrettBaseHW {
public:
  WamHW(::barrett::systems::Wam<DOF> *wam,
        boost::array<std::string, DOF> const *joint_names = NULL)
    : state_position_(0.)
    , state_velocity_(0.)
    , state_effort_(0.)
    , command_effort_(0.)
    , wam_(wam)
    , llwam_(&wam_->getLowLevelWam())
  {
    if (joint_names) {
      joint_names_ = *joint_names;
    } else {
      for (size_t i = 0; i < DOF; ++i) {
        joint_names_[i] = boost::str(boost::format("wam_joint%d") % (i + 1));
      }
    }
  }

  virtual ~WamHW()
  {
  }

  virtual void registerHandles(BarrettInterfaces &interfaces)
  {
    for (size_t i = 0; i < DOF; ++i) {
      ::hardware_interface::JointStateHandle jointstate_handle(
        joint_names_[i],
        &state_position_[i], &state_velocity_[i], &state_effort_[i]
      );
      interfaces.jointstate_interface.registerHandle(jointstate_handle);

      interfaces.jointeffort_interface.registerHandle(
        ::hardware_interface::JointHandle(
          jointstate_handle, &command_effort_[i]
        )
      );
    }
  }

  virtual void read()
  {
    llwam_->update();

    state_position_ = wam_->getJointPositions();
    state_velocity_ = wam_->getJointVelocities();
    state_effort_ = wam_->getJointTorques();
  }

  virtual void write()
  {
    // TODO: I'm not exactly what the difference is between "Wam",
    // "LowLevelWamWrapper", and "LowLevelWam". How do I correctly write
    // torques here that: (1) include gravity compensation and (2) prevent the
    // pucks from heartbeat faulting.
    llwam_->setTorques(command_effort_);
  }

private:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  boost::array<std::string, DOF> joint_names_;

  jp_type state_position_;
  jv_type state_velocity_;
  jt_type state_effort_;
  jt_type command_effort_;

  ::barrett::systems::Wam<DOF> *wam_;
  ::barrett::LowLevelWam<DOF> *llwam_;

  DISALLOW_COPY_AND_ASSIGN(WamHW);
};

}

#endif
