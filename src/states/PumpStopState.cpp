#include "PumpStopState.h"

#include <devices/Pump.h>

namespace mc_panda
{

void PumpStopState::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
}

void PumpStopState::start(mc_control::fsm::Controller & ctl_)
{
  auto & robot = robot_.empty() ? ctl_.robot() : ctl_.robot(robot_);
  auto pump_ = Pump::get(robot);
  if(!pump_)
  {
    mc_rtc::log::warning("[{}] State started with robot {} which does not have an mc_panda::Pump device", name(),
                         robot_);
    output("NoPump");
    return;
  }
  pump_->stop();
  output("OK");
}

bool PumpStopState::run(mc_control::fsm::Controller & ctl_)
{
  return true;
}

void PumpStopState::teardown(mc_control::fsm::Controller & ctl_) {}

} // namespace mc_panda

EXPORT_SINGLE_STATE("PumpStop", mc_panda::PumpStopState)
