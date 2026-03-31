#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

namespace mc_panda
{

/** Send a stop command to the pump
 *
 * If the robot has no pump the state always outputs "NoPump"
 *
 * Otherwise the state always outputs "OK"
 */
struct PumpStopState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string robot_; // robot which has the pump
};

} // namespace mc_panda
