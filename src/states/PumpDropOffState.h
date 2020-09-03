#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <devices/Pump.h>

namespace mc_panda
{

/** Send a drop off command and (optionally) wait for the command completion
 *
 * If the robot has no pump the state always outputs "NoPump"
 *
 * If the command fails the state outputs "DropOffFailure" otherwise the state outputs "OK"
 *
 * If the state is configured not to wait for the command completion, this always outputs "OK"
 *
 * If the pump is busy when this state starts it will wait for the pump to finish the previous command
 *
 */
struct PumpDropOffState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void stop(mc_control::fsm::Controller & ctl) override;

private:
  std::string robot_; // robot which has the pump
  uint64_t timeout_ = 1000; // drop timeout (milliseconds)
  bool waiting_ = true; // if true do not wait for the command completion

  Pump * pump_;
  bool requested_ = false;
  bool done_ = false;

  bool request();
};

} // namespace mc_panda
