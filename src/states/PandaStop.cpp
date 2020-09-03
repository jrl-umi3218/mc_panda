#include "PandaStop.h"

#include <mc_control/fsm/Controller.h>

#include <devices/Robot.h>

namespace mc_panda
{

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
  auto robot = Robot::get(ctl_.robot(robot_));
  if(robot)
  {
    robot->stop();
  }
  else
  {
    mc_rtc::log::warning("[{}] State started with robot {} which does not have an mc_panda::Robot device", name(),
                         robot_);
  }
  output("OK");
}

bool PandaStop::run(mc_control::fsm::Controller & ctl_)
{
  return true;
}

void PandaStop::teardown(mc_control::fsm::Controller & ctl_) {}

} // namespace mc_panda

EXPORT_SINGLE_STATE("PandaStop", mc_panda::PandaStop)
