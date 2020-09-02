#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_panda/devices/Pump.h>

struct PumpStopState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration state_conf_;

  const std::string pumpDeviceName = "Pump";
  const std::string robname = "panda_pump";
  bool command_requested = false;
};
