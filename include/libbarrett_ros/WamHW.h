#ifndef LIBBARRETT_ROS_WAMHW_H_
#define LIBBARRETT_ROS_WAMHW_H_
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <barrett/systems/wam.h>
#include <libbarrett_ros/BarrettBaseHW.h>

namespace libbarrett_ros {

template <size_t DOF>
class WamHW : public BarrettBaseHW {
public:
  WamHW(barrett::systems::Wam<DOF> *wam,
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

  virtual void requestCritical()
  {
    using ::barrett::Puck;
    using ::barrett::PuckGroup;

    PuckGroup const &group = llwam_->getPuckGroup();
    int const P_id = group.getPropertyId(Puck::P);

    group.sendGetPropertyRequest(P_id);
  }

  virtual void receiveCritical()
  {
    using ::barrett::Puck;
    using ::barrett::PuckGroup;
    using ::barrett::MotorPuck;

    static bool const realtime = false;

    // Receive the puck positions.
    PuckGroup const &group = llwam_->getPuckGroup();
    int const P_id = group.getPropertyId(Puck::P);
    
    group.receiveGetPropertyReply<MotorPuck::MotorPositionParser<double> >(
        P_id, state_motor_position_.data(), realtime);

    // Convert from puck positions to motor positions.
    state_position_ = llwam_->getPuckToJointPositionTransform()
                      * state_motor_position_;
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
    // TODO: This does not include gravity compensation.
    llwam_->setTorques(command_effort_);
  }

private:
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  boost::array<std::string, DOF> joint_names_;

  jp_type state_position_;
  jp_type state_motor_position_;
  jv_type state_velocity_;
  jt_type state_effort_;
  jt_type command_effort_;

  ::barrett::systems::Wam<DOF> *wam_;
  ::barrett::LowLevelWam<DOF> *llwam_;

  DISALLOW_COPY_AND_ASSIGN(WamHW);
};

}

#endif
