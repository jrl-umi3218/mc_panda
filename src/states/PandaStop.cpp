#include "PandaStop.h"
#include <mc_tasks/MetaTaskLoader.h>

void PandaStop::configure(const mc_rtc::Configuration & config)
{
}

void PandaStop::start(mc_control::fsm::Controller & ctl_)
{
  if(ctl_.robot().hasDevice<mc_panda::PandaSensor>(sensorDeviceName))
  {
    sensorAvailable = true;
    // mc_rtc::log::info("RobotModule has a PandaSensor named {}", sensorDeviceName);
  }
  // else{
  //   mc_rtc::log::warning("RobotModule does not have a PandaSensor named {}", sensorDeviceName);
  //   mc_rtc::log::warning("PandaSensor functionality will not be available");
  // }

  // mc_rtc::log::success("PandaStop state start done");
}

bool PandaStop::run(mc_control::fsm::Controller & ctl_)
{
  if(sensorAvailable){
    ctl_.robot().device<mc_panda::PandaSensor>(sensorDeviceName).requestStopCommand();
  }
  return false;
}

void PandaStop::teardown(mc_control::fsm::Controller & ctl_)
{
  // mc_rtc::log::info("PandaStop teardown done");
}

EXPORT_SINGLE_STATE("PandaStop", PandaStop)
