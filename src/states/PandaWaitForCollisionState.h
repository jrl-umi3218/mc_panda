#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <devices/Robot.h>

namespace mc_panda
{

namespace details
{

/** Returns an array filled with infinity */
template<size_t N>
std::array<double, N> default_thresholds()
{
  std::array<double, N> out;
  out.fill(std::numeric_limits<double>::infinity());
  return out;
}

} // namespace details

/** Wait for the panda robot to report a collision, using any of:
 * - a joint contact threshold
 * - a cartesian contact threshold
 * - a pressure threshold
 *
 * The states outputs "NotAPanda" if the robot does not have an mc_panda::Robot attached
 *
 * Otherwise the state outputs OK when the contact is detected, i.e. when any of the contact values/pressure exceeds the
 * corresponding threshold.
 */
struct PandaWaitForCollisionState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void addToLogger(mc_rtc::Logger & logger);
  void removeFromLogger(mc_rtc::Logger & logger);

  std::string robot_;
  std::array<double, 7> joint_contact_thresholds_ = details::default_thresholds<7>();
  std::array<double, 6> cartesian_contact_thresholds_ = details::default_thresholds<6>();
  double pressure_threshold_ = std::numeric_limits<double>::infinity();

  Robot * device_;
};

} // namespace mc_panda
