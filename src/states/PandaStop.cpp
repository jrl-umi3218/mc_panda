#include "PandaStop.h"

#include <mc_control/fsm/Controller.h>

#include <devices/PandaDevice.h>

void PandaStop::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
}

void PandaStop::start(mc_control::fsm::Controller & ctl_)
{
  if(robot_.size() == 0)
  {
    robot_ = ctl_.robot().name();
  }
  auto & robot = ctl_.robot(robot_);
  const auto & device = mc_panda::PandaDevice::name;
  sensorAvailable_ = robot.hasDevice<mc_panda::PandaDevice>(device);
  if(sensorAvailable_)
  {
    ctl_.robot().device<mc_panda::PandaDevice>(device).stop();
  }
  output("OK");
}

bool PandaStop::run(mc_control::fsm::Controller & ctl_)
{
  return true;
}

void PandaStop::teardown(mc_control::fsm::Controller & ctl_) {}

EXPORT_SINGLE_STATE("PandaStop", PandaStop)
