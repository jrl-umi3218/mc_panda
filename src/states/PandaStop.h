#pragma once

#include <mc_control/fsm/State.h>

/** Stop all commands execution on the related panda robot */
struct PandaStop : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string robot_;
  bool sensorAvailable_;
};
