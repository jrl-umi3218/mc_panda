#include "Pump.h"

#include <mc_rbdyn/Robot.h>

#include <franka/exception.h>

namespace mc_panda
{

Pump::Pump(const std::string & parent, const sva::PTransformd & X_p_d) : mc_rbdyn::Device(Pump::name, parent, X_p_d)
{
  type_ = "Pump";
}

Pump::~Pump()
{
  disconnect();
}

Pump * Pump::get(mc_rbdyn::Robot & robot)
{
  if(robot.hasDevice<Pump>(Pump::name))
  {
    return &(robot.device<Pump>(Pump::name));
  }
  return nullptr;
}

mc_rbdyn::DevicePtr Pump::clone() const
{
  if(gripper_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot copy a connected Pump");
  }
  auto ret = new Pump(parent_, X_p_s_);
  ret->state_ = this->state_;
  return mc_rbdyn::DevicePtr(ret);
}

void Pump::addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.addLogEntry(prefix + "_status", [this]() { return state_.device_status != Status::disconnected; });
  logger.addLogEntry(prefix + "_device_status", [this]() { return static_cast<StatusInt>(state_.device_status); });
  logger.addLogEntry(prefix + "_actual_power", [this]() { return state_.actual_power; });
  logger.addLogEntry(prefix + "_vacuum", [this]() { return state_.vacuum; });
  logger.addLogEntry(prefix + "_part_detached", [this]() { return state_.part_detached; });
  logger.addLogEntry(prefix + "_part_present", [this]() { return state_.part_present; });
  logger.addLogEntry(prefix + "_in_control_range", [this]() { return state_.in_control_range; });
  logger.addLogEntry(prefix + "_last_command_id", [this]() { return last_command_id_; });
}

void Pump::removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.removeLogEntry(prefix + "_status");
  logger.removeLogEntry(prefix + "_device_status");
  logger.removeLogEntry(prefix + "_actual_power");
  logger.removeLogEntry(prefix + "_vacuum");
  logger.removeLogEntry(prefix + "_part_detached");
  logger.removeLogEntry(prefix + "_part_present");
  logger.removeLogEntry(prefix + "_in_control_range");
  logger.removeLogEntry(prefix + "_last_command_id");
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
      if(interrupted_)
      {
        bool busy = busy_;
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
        success_ = s;
        error_ = error;
        if(!busy)
        {
          interrupted_ = false;
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

bool Pump::vacuum(uint8_t vacuum, std::chrono::milliseconds timeout, ProductionSetupProfile profile)
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
  command_ = {"vacuum", [=]() { return gripper_->vacuum(vacuum, timeout, profile); }};
  last_command_id_ = 1;
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
  command_ = {"dropOff", [=]() { return gripper_->dropOff(timeout); }};
  last_command_id_ = 2;
  return true;
}

bool Pump::stop()
{
  if(!gripper_)
  {
    return true;
  }
  if(interrupted_)
  {
    mc_rtc::log::error("{} stop command has already been requested", name_);
    return false;
  }
  interrupted_ = true;
  last_command_id_ = 3;
  return true;
}

} // namespace mc_panda
