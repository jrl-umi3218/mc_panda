#include "PumpDropOffState.h"

namespace mc_panda
{

void PumpDropOffState::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
  config("timeout", timeout_);
  config("waiting", waiting_);
}

void PumpDropOffState::start(mc_control::fsm::Controller & ctl_)
{
  auto & robot = robot_.empty() ? ctl_.robot() : ctl_.robot(robot_);
  pump_ = Pump::get(robot);
  if(!pump_)
  {
    mc_rtc::log::warning("[{}] State started with robot {} which does not have an mc_panda::Pump device", name(),
                         robot_);
    output("NoPump");
    return;
  }
  request();
}

bool PumpDropOffState::run(mc_control::fsm::Controller & ctl_)
{
  if(!pump_ || (!waiting_ && requested_) || done_)
  {
    return true;
  }

  // Pump still busy from a previous command
  if(!requested_)
  {
    return request();
  }
  // Pump busy from this command
  if(pump_->busy())
  {
    return false;
  }

  done_ = true;
  if(pump_->success())
  {
    output("OK");
  }
  else
  {
    pump_->stop();
    output("DropOffFailure");
  }
  return true;
}

void PumpDropOffState::teardown(mc_control::fsm::Controller & ctl_) {}

bool PumpDropOffState::request()
{
  if(!pump_->busy())
  {
    requested_ = pump_->dropOff(std::chrono::milliseconds(timeout_));
    if(requested_)
    {
      mc_rtc::log::info("[{}] Pump dropOff requested with timeout: {}ms", name(), timeout_);
      if(!waiting_)
      {
        output("OK");
      }
    }
  }
  return requested_ && !waiting_;
}

void PumpDropOffState::stop(mc_control::fsm::Controller & ctl)
{
  if(pump_)
  {
    mc_rtc::log::info("[{}] Send interrupt command to the pump", name());
    pump_->stop();
  }
}

} // namespace mc_panda

EXPORT_SINGLE_STATE("PumpDropOff", mc_panda::PumpDropOffState)
