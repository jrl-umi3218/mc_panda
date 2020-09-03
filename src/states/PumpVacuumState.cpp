#include "PumpVacuumState.h"

namespace mc_panda
{

void PumpVacuumState::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
  config("vacuum", vacuum_);
  config("timeout", timeout_);
  config("waiting", waiting_);
  config("profile", profile_);
}

void PumpVacuumState::start(mc_control::fsm::Controller & ctl_)
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
  auto max_vacuum = std::numeric_limits<uint8_t>::max();
  if(vacuum_ > max_vacuum)
  {
    mc_rtc::log::warning("[{}] Configured pressure is {} but maximum allowed pressure is {}", vacuum_, max_vacuum);
    vacuum_ = max_vacuum;
  }
  request();
}

bool PumpVacuumState::run(mc_control::fsm::Controller & ctl_)
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
    output("VacuumFailure");
  }
  return true;
}

void PumpVacuumState::teardown(mc_control::fsm::Controller & ctl_) {}

bool PumpVacuumState::request()
{
  if(!pump_->busy())
  {
    requested_ = pump_->vacuum(vacuum_, std::chrono::milliseconds(timeout_), profile_);
    if(requested_)
    {
      mc_rtc::log::info("[{}] Pump vacuum requested with timeout: {}ms, vacuum strength: {}mbar", name(), timeout_,
                        10 * vacuum_);
      if(!waiting_)
      {
        output("OK");
      }
    }
  }
  return requested_ && !waiting_;
}

void PumpVacuumState::stop(mc_control::fsm::Controller & ctl)
{
  if(pump_)
  {
    mc_rtc::log::info("[{}] Send interrupt command to the pump", name());
    pump_->stop();
  }
}

} // namespace mc_panda

EXPORT_SINGLE_STATE("PumpVacuum", mc_panda::PumpVacuumState)
