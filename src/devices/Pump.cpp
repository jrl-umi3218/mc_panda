#include "Pump.h"

#include <franka/exception.h>

namespace mc_panda
{

Pump::Pump(const std::string & name, const std::string & parent, const sva::PTransformd & X_p_d)
: mc_rbdyn::Device(name, parent, X_p_d)
{
  type_ = "Pump";
}

Pump::~Pump()
{
  disconnect();
}

void Pump::name(const std::string & name)
{
  name_ = name;
}

mc_rbdyn::DevicePtr Pump::clone() const
{
  if(gripper_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot copy a connected Pump");
  }
  auto ret = new Pump(name_, parent_, X_p_s_);
  ret->state_ = this->state_;
  return mc_rbdyn::DevicePtr(ret);
}

void Pump::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_status", [this]() { return state_.device_status != Status::disconnected; });
  logger.addLogEntry(name_ + "_device_status", [this]() { return static_cast<StatusInt>(state_.device_status); });
  logger.addLogEntry(name_ + "_actual_power", [this]() { return state_.actual_power; });
  logger.addLogEntry(name_ + "_vacuum", [this]() { return state_.vacuum; });
  logger.addLogEntry(name_ + "_part_detached", [this]() { return state_.part_detached; });
  logger.addLogEntry(name_ + "_part_present", [this]() { return state_.part_present; });
  logger.addLogEntry(name_ + "_in_control_range", [this]() { return state_.in_control_range; });
}

void Pump::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_status");
  logger.removeLogEntry(name_ + "_device_status");
  logger.removeLogEntry(name_ + "_actual_power");
  logger.removeLogEntry(name_ + "_vacuum");
  logger.removeLogEntry(name_ + "_part_detached");
  logger.removeLogEntry(name_ + "_part_present");
  logger.removeLogEntry(name_ + "_in_control_range");
}

bool Pump::connect(const std::string & ip)
{
  if(gripper_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is already connected", name_);
  }
  try
  {
    gripper_.reset(new franka::VacuumGripper(ip));
    mc_rtc::log::info("{} connected to {}", name_, ip);
  }
  catch(const franka::NetworkException & exc)
  {
    mc_rtc::log::error("{} failed to connect to {}: {}", name_, ip, exc.what());
    return false;
  }
  catch(const franka::IncompatibleVersionException & exc)
  {
    mc_rtc::log::error("{} imcompatible version with {}: {}", name_, ip, exc.what());
    return false;
  }
  connected_ = true;
  stateThread_ = std::thread([this]() {
    while(connected_)
    {
      try
      {
        auto stateIn = gripper_->readOnce();
        std::unique_lock<std::mutex> lock(stateMutex_);
        state_ = stateIn;
        status_ = Status(static_cast<StatusInt>(state_.device_status));
      }
      catch(const franka::NetworkException & exc)
      {
        mc_rtc::log::error("{} connection lost, failed to read state: {}", name_, exc.what());
      }
    }
  });
  commandThread_ = std::thread([this]() {
    while(connected_)
    {
      if(busy_)
      {
        bool s = false;
        std::string error;
        try
        {
          s = command_.callback();
          error = "";
        }
        catch(const franka::CommandException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} {} command failed: {}", name_, command_.name, error);
        }
        catch(const franka::NetworkException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} connection lost, failed to execute {} command: {}", name_, command_.name, error);
        }
        if(!interrupted_)
        {
          success_ = s;
          error_ = error;
        }
        else
        {
          interrupted_ = false;
        }
        busy_ = false;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });
  interruptThread_ = std::thread([this]() {
    while(connected_)
    {
      if(busy_ && interrupted_)
      {
        bool s = false;
        std::string error;
        try
        {
          s = gripper_->stop();
          error = "";
        }
        catch(const franka::CommandException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} stop command failed: {}", name_, error);
        }
        catch(const franka::NetworkException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} connection lost, failed to execute stop command: {}", name_, error);
        }
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });
  return true;
}

void Pump::disconnect()
{
  if(!gripper_)
  {
    return;
  }
  connected_ = false;
  commandThread_.join();
  stateThread_.join();
  interruptThread_.join();
  gripper_.reset(nullptr);
}

const franka::VacuumGripperState & Pump::state() const
{
  std::unique_lock<std::mutex> lock(stateMutex_);
  return state_;
}

auto Pump::status() const -> Status
{
  std::unique_lock<std::mutex> lock(stateMutex_);
  return status_;
}

bool Pump::busy() const
{
  return busy_;
}

bool Pump::success() const
{
  return success_;
}

const std::string & Pump::error() const
{
  return error_;
}

bool Pump::vacuum(uint8_t vacuum,
                  std::chrono::milliseconds timeout,
                  ProductionSetupProfile profile)
{
  if(!gripper_)
  {
    return true;
  }
  if(busy_)
  {
    mc_rtc::log::error("{} is already busy executing {} command", name_, command_.name);
    return false;
  }
  busy_ = true;
  command_ = {
    "vacuum",
    [=]() { return gripper_->vacuum(vacuum, timeout, profile); }
  };
  return true;
}

bool Pump::dropOff(std::chrono::milliseconds timeout)
{
  if(!gripper_)
  {
    return true;
  }
  if(busy_)
  {
    mc_rtc::log::error("{} is already busy executing {} command", name_, command_.name);
    return false;
  }
  busy_ = true;
  command_ = {
    "dropOff",
    [=]() { return gripper_->dropOff(timeout); }
  };
  return true;
}

bool Pump::stop()
{
  if(!gripper_)
  {
    return true;
  }
  if(!busy_)
  {
    mc_rtc::log::error("No command is being executed on {}, nothing to stop", name_);
    return false;
  }
  if(interrupted_)
  {
    mc_rtc::log::error("{} stop command has already been requested", name_);
    return false;
  }
  interrupted_ = true;
  return true;
}

} // namespace mc_panda
