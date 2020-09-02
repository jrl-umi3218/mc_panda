#include "PumpVacuumState.h"
#include <mc_tasks/MetaTaskLoader.h>

void PumpVacuumState::configure(const mc_rtc::Configuration & config)
{
  mc_rtc::log::success("PumpVacuumState configure done");
}

void PumpVacuumState::start(mc_control::fsm::Controller & ctl_)
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
  mc_rtc::log::success("PumpVacuumState start done");
}

bool PumpVacuumState::run(mc_control::fsm::Controller & ctl_)
{
  // franka::VacuumGripperState state = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).state();
  // mc_panda::Pump::Status status = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).status();

  bool busy = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).busy();
  if(busy)
  {
    mc_rtc::log::warning("pump is busy, can not request vacuum command");
    return false;
  }
  if(command_requested)
  {
    if(succeedImmediately)
    {
      output("OK");
      return true;
    }
    else
    {
      bool success = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).success();
      if(success)
      {
        mc_rtc::log::warning("pump vacuum command was successful");
        output("OK");
        return true;
      }
      else
      {
        // try again if not successful
        command_requested = false;
        std::string error = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).error();
        mc_rtc::log::warning("pump vacuum command not successful: {}", error);
        return false;
      }
    }
  }
  else
  {
    const uint8_t vacuum = 100; // unit [10*mbar]
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(2000); // unit ms
    mc_rtc::log::info("requesting pump vacuum command: vacuum={}, timeout={}", vacuum, std::to_string(timeout.count()));
    ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).vacuum(vacuum, timeout);
    command_requested = true;
    mc_rtc::log::info("pump vacuum command requested");
    return false;
  }
  return false;
}

void PumpVacuumState::teardown(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("PumpVacuumState teardown done");
}

EXPORT_SINGLE_STATE("PumpVacuumState", PumpVacuumState)
