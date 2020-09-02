#include "PumpDropoffState.h"

#include <devices/Pump.h>

void PumpDropoffState::configure(const mc_rtc::Configuration & config)
{
  mc_rtc::log::success("PumpDropoffState configure done");
}

void PumpDropoffState::start(mc_control::fsm::Controller & ctl_)
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
  mc_rtc::log::success("PumpDropoffState start done");
}

bool PumpDropoffState::run(mc_control::fsm::Controller & ctl_)
{
  // franka::VacuumGripperState state = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).state();
  // mc_panda::Pump::Status status = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).status();

  bool busy = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).busy();
  if(busy)
  {
    mc_rtc::log::warning("pump is busy, can not request dropoff command");
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
        mc_rtc::log::warning("pump dropoff command was successful");
        output("OK");
        return true;
      }
      else
      {
        // try again if not successful
        command_requested = false;
        std::string error = ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).error();
        mc_rtc::log::warning("pump dropoff command not successful: {}", error);
        return false;
      }
    }
  }
  else
  {
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(1000); // unit ms
    mc_rtc::log::info("requesting pump dropoff command: timeout={}", std::to_string(timeout.count()));
    ctl_.robot(robname).device<mc_panda::Pump>(pumpDeviceName).dropOff(timeout);
    command_requested = true;
    mc_rtc::log::info("pump dropoff command requested");
    return false;
  }
  return false;
}

void PumpDropoffState::teardown(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("PumpDropoffState teardown done");
}

EXPORT_SINGLE_STATE("PumpDropoffState", PumpDropoffState)
