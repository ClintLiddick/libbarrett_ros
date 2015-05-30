#include <libbarrett_ros/HandHW.h>

using libbarrett_ros::BarrettInterfaces;
using libbarrett_ros::HandHW;

HandHW::HandHW(barrett::Hand *hand)
  : hand_(hand)
  , realtime_(false)
{
}

void HandHW::registerHandles(BarrettInterfaces &interfaces)
{
  for (size_t ifinger = 0; ifinger < NUM_FINGERS; ++ifinger) {
    std::stringstream inner_name, outer_name;
    inner_name << "j" << ifinger << "0";
    outer_name << "j" << ifinger << "1";

    hardware_interface::JointStateHandle jointstate_inner_handle(
      inner_name.str(),
      &state_[ifinger].inner.position,
      &state_[ifinger].inner.velocity,
      &state_[ifinger].inner.effort
    );
    interfaces.jointstate_interface.registerHandle(jointstate_inner_handle);

    hardware_interface::JointStateHandle jointstate_outer_handle(
      outer_name.str(),
      &state_[ifinger].outer.position,
      &state_[ifinger].outer.velocity,
      &state_[ifinger].outer.effort
    );
    interfaces.jointstate_interface.registerHandle(jointstate_outer_handle);
  }

  hardware_interface::JointStateHandle jointstate_spread_handle(
    "j00",
    &spread_state_.position,
    &spread_state_.velocity,
    &spread_state_.effort
  );
  interfaces.jointstate_interface.registerHandle(jointstate_spread_handle);
}

void HandHW::requestCritical()
{
  using barrett::Puck;
  using barrett::PuckGroup;

  PuckGroup const &group = hand_->getPuckGroup();

  group.sendGetPropertyRequest(group.getPropertyId(Puck::P));
}

void HandHW::receiveCritical()
{
  using barrett::Puck;
  using barrett::PuckGroup;
  using barrett::MotorPuck;
  using barrett::Hand;

  static double const J2_RATIO = 125.0;
  static double const J2_ENCODER_RATIO = 50.0;
  static double const J3_RATIO = 375.0;
  static double const SPREAD_RATIO = 17.5;

  PuckGroup const &group = hand_->getPuckGroup();
  std::vector<MotorPuck> const &motor_pucks = hand_->getMotorPucks();

  group.receiveGetPropertyReply<MotorPuck::CombinedPositionParser<int> >(
    group.getPropertyId(Puck::P), position_raw_.data(), realtime_);
  
  for (size_t i = 0; i < NUM_FINGERS; ++i) {
    MotorPuck const &motor_puck = motor_pucks[i];
    double const &primary_encoder = position_raw_[i].get<0>();
    double const &secondary_encoder = position_raw_[i].get<1>();

    // Use the secondary encoder, if available.
    if (secondary_encoder != std::numeric_limits<int>::max()) {
      state_[i].inner.position = motor_puck.counts2rad(secondary_encoder)
        / J2_ENCODER_RATIO;
      state_[i].outer.position = motor_puck.counts2rad(primary_encoder)
        * (1.0 / J2_RATIO + 1.0 / J3_RATIO) - state_[i].inner.position;
    }
    // Otherwise fall back on only using the primary encoder. These
    // calculations are only valid before breakaway.
    else {
      state_[i].inner.position
        = motor_puck.counts2rad(primary_encoder) / J2_RATIO;
      state_[i].outer.position
        = state_[i].inner.position * J2_RATIO / J3_RATIO;
    }
  }

  spread_state_.position = motor_pucks[Hand::SPREAD_INDEX].counts2rad(
    position_raw_[Hand::SPREAD_INDEX].get<0>()) / SPREAD_RATIO;
}

void HandHW::requestOther()
{
  using barrett::Puck;
  using barrett::PuckGroup;
  using barrett::TactilePuck;

  PuckGroup const &group = hand_->getPuckGroup();


#if 0
  if (hand_->hasFingertipTorqueSensors()) {
		group.sendGetPropertyRequest(group.getPropertyId(Puck::SG));
  }

	if (hand_->hasTactSensors()) {
		group.setProperty(Puck::TACT, TactilePuck::FULL_FORMAT);
	}
#endif
}

void HandHW::receiveOther()
{
}

void HandHW::write()
{
}

void HandHW::halt()
{
}
