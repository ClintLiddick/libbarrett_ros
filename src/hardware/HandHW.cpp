#include <libbarrett_ros/hardware/HandHW.h>

using libbarrett_ros::BarrettInterfaces;
using libbarrett_ros::HandHW;

HandHW::HandHW(barrett::Hand *hand)
  : hand_(hand)
  , realtime_(false)
{
}

void HandHW::registerHandles(BarrettInterfaces &interfaces)
{ for (size_t ifinger = 0; ifinger < NUM_FINGERS; ++ifinger) {
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

  // NOTE: OWD also request TEMP, THERM, and MODE.

  if (!strain_pending_.empty() && hand_->hasFingertipTorqueSensors()) {
		group.sendGetPropertyRequest(group.getPropertyId(Puck::SG));

    for (size_t i = 0; i < group.numPucks(); ++i) {
      strain_pending_.push_back(i);
    }
  }

  // TODO: Tactile sensors are disabled to save bandwidth.
#if 0
	if (hand_->hasTactSensors()) {
		group.setProperty(Puck::TACT, TactilePuck::FULL_FORMAT);
	}
#endif
}

void HandHW::receiveOther()
{
  using barrett::Puck;
  using barrett::PuckGroup;

  // Wait for an SG reply from one or more finger pucks.
  if (!strain_pending_.empty()) {
    PuckGroup const &group = hand_->getPuckGroup();
    barrett::bus::CommunicationsBus const &bus = group.getPucks()[0]->getBus();
    int const prop_id = group.getPropertyId(Puck::SG);

    std::list<size_t>::iterator it = strain_pending_.begin();
    while (it != strain_pending_.end()) {
      size_t const finger_index = *it;
      int const ret = Puck::receiveGetPropertyReply<Puck::StandardParser>(
        bus, group.getPuckId(finger_index),
        prop_id, &strain_[finger_index],
        false, realtime_
      );

      // We received a response from this puck; stop waiting for it.
      if (ret == 0) {
        it = strain_pending_.erase(it);
      }
      // Timed out. Check again in the next control cycle.
      else if (ret == 1) {
        ++it;
      } else {
        throw std::runtime_error("Failed to receive acceleration.");
      }
    }
  }
}

void HandHW::write()
{
  // TODO: How do we want to control the hand? Torques? Or high-level commands?
}

void HandHW::halt()
{
}
