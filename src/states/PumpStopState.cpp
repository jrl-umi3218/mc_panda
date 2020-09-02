#include "PumpStopState.h"

#include <devices/Pump.h>

void PumpStopState::configure(const mc_rtc::Configuration & config)
{
  mc_rtc::log::success("PumpStopState configure done");
}

void PumpStopState::start(mc_control::fsm::Controller & ctl_)
{
  if(ctl_.robot(robname).hasDevice<mc_panda::Pump>(pumpDeviceName))
  {
    mc_rtc::log::info("RobotModule has a Pump named {}", pumpDeviceName);
  }
  else
  {
    mc_rtc::log::warning("RobotModule does not have a Pump named {}", pumpDeviceName);
    mc_rtc::log::error_and_throw<std::runtime_error>("Pump functionality is not available");
  }
  mc_rtc::log::success("PumpStopState start done");
}

bool PumpStopState::run(mc_control::fsm::Controller & ctl_)
{
  // franka::VacuumGripperState state = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).state();
  // mc_panda::Pump::Status status = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).status();

  bool busy = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).busy();
  if(busy)
  {
    return false;
  }
  if(command_requested)
  {
    bool success = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).success();
    if(success)
    {
      output("OK");
      return true;
    }
    else
    {
      // try again if not successful
      command_requested = false;
      std::string error = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).error();
      mc_rtc::log::warning("pump stop command not successful: {}", error);
      return false;
    }
  }
  else
  {
    mc_rtc::log::info("requesting pump stop command");
    ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).stop();
    command_requested = true;
    mc_rtc::log::info("pump stop command requested");
    return false;
  }
  return false;
}

void PumpStopState::teardown(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("PumpStopState teardown done");
}

EXPORT_SINGLE_STATE("PumpStopState", PumpStopState)
