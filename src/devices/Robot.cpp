#include "Robot.h"

#include <franka/exception.h>

namespace mc_panda
{

Robot::~Robot()
{
  if(robot_)
  {
    disconnect();
  }
}

mc_rbdyn::DevicePtr Robot::clone() const
{
  if(robot_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot clone a connected Robot");
  }
  auto device = new Robot();
  device->state_ = state_;
  return mc_rbdyn::DevicePtr(device);
}

void Robot::addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.addLogEntry(prefix + "_tau_ext_hat_filtered",
                     [this]() -> const std::array<double, 7> & { return state_.tau_ext_hat_filtered; });
  logger.addLogEntry(prefix + "_O_F_ext_hat_K",
                     [this]() -> const std::array<double, 6> & { return state_.O_F_ext_hat_K; });
  logger.addLogEntry(prefix + "_K_F_ext_hat_K",
                     [this]() -> const std::array<double, 6> & { return state_.K_F_ext_hat_K; });
  logger.addLogEntry(prefix + "_control_command_success_rate",
                     [this]() { return state_.control_command_success_rate; });
  logger.addLogEntry(prefix + "_joint_contact",
                     [this]() -> const std::array<double, 7> & { return state_.joint_contact; });
  logger.addLogEntry(prefix + "_cartesian_contact",
                     [this]() -> const std::array<double, 6> & { return state_.cartesian_contact; });
  logger.addLogEntry(prefix + "_torque_derivatives",
                     [this]() -> const std::array<double, 7> & { return state_.dtau_J; });
  // FIXME Previous version logged singular values which does not match anythin in franka::RobotState
}

void Robot::removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.removeLogEntry(prefix + "_tau_ext_hat_filtered");
  logger.removeLogEntry(prefix + "_O_F_ext_hat_K");
  logger.removeLogEntry(prefix + "_K_F_ext_hat_K");
  logger.removeLogEntry(prefix + "_control_command_success_rate");
  logger.removeLogEntry(prefix + "_joint_contact");
  logger.removeLogEntry(prefix + "_cartesian_contact");
  logger.removeLogEntry(prefix + "_torque_derivatives");
}

void Robot::connect(franka::Robot * robot)
{
  std::unique_lock<std::mutex> lock(robotMutex_);
  if(robot_)
  {
    lock.unlock();
    disconnect();
    lock.lock();
  }
  robot_ = robot;
  commandThread_ = std::thread([this]() {
    while(robot_)
    {
      {
        std::unique_lock<std::mutex> lock(commandMutex_);
        commandCv_.wait(lock, [this]() { return commands_.size() || !robot_; });
      }
      std::unique_lock<std::mutex> rLock(robotMutex_);
      while(robot_ && commands_.size())
      {
        auto & cmd = commands_.front();
        try
        {
          cmd.command();
        }
        catch(const franka::CommandException & exc)
        {
          mc_rtc::log::error("[{}] Command exception while executing {}:\n{}", name_, cmd.name, exc.what());
        }
        catch(const franka::NetworkException & exc)
        {
          mc_rtc::log::error("[{}] Network exception while executing {}:\n{}", name_, cmd.name, exc.what());
        }
        std::unique_lock<std::mutex> lock(commandMutex_);
        commands_.pop();
      }
    }
    std::unique_lock<std::mutex> lock(commandMutex_);
    while(commands_.size())
    {
      commands_.pop();
    }
  });
}

void Robot::disconnect()
{
  if(!robot_)
  {
    return;
  }
  {
    std::unique_lock<std::mutex> lock(commandMutex_, std::defer_lock);
    std::unique_lock<std::mutex> rLock(robotMutex_, std::defer_lock);
    std::lock(lock, rLock);
    robot_ = nullptr;
  }
  commandCv_.notify_one();
  commandThread_.join();
}

void Robot::setLoad(double load_mass,
                    const std::array<double, 3> & F_x_Cload,
                    const std::array<double, 9> & load_inertia)
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("setLoad", [=]() { robot_->setLoad(load_mass, F_x_Cload, load_inertia); });
  }
  commandCv_.notify_one();
}

void Robot::setCollisionBehavior(const std::array<double, 7> & lower_torque_thresholds_acceleration,
                                 const std::array<double, 7> & upper_torque_thresholds_acceleration,
                                 const std::array<double, 7> & lower_torque_thresholds_nominal,
                                 const std::array<double, 7> & upper_torque_thresholds_nominal,
                                 const std::array<double, 6> & lower_force_thresholds_acceleration,
                                 const std::array<double, 6> & upper_force_thresholds_acceleration,
                                 const std::array<double, 6> & lower_force_thresholds_nominal,
                                 const std::array<double, 6> & upper_force_thresholds_nominal)
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("setCollisionBehavior", [=]() {
      robot_->setCollisionBehavior(lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                                   lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                   lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                                   lower_force_thresholds_nominal, upper_force_thresholds_nominal);
    });
  }
  commandCv_.notify_one();
}

void Robot::setCollisionBehavior(const std::array<double, 7> & lower_torque_thresholds,
                                 const std::array<double, 7> & upper_torque_thresholds,
                                 const std::array<double, 6> & lower_force_thresholds,
                                 const std::array<double, 6> & upper_force_thresholds)
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("setCollisionBehavior", [=]() {
      robot_->setCollisionBehavior(lower_torque_thresholds, upper_torque_thresholds, lower_force_thresholds,
                                   upper_force_thresholds);
    });
  }
  commandCv_.notify_one();
}

void Robot::setJointImpedance(const std::array<double, 7> & K_theta)
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("setJointImpedance", [=]() { robot_->setJointImpedance(K_theta); });
  }
  commandCv_.notify_one();
}

void Robot::setCartesianImpedance(const std::array<double, 6> & K_x)
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("setCartesianImpedance", [=]() { robot_->setCartesianImpedance(K_x); });
  }
  commandCv_.notify_one();
}

void Robot::stop()
{
  {
    std::unique_lock<std::mutex> lock(commandMutex_);
    commands_.emplace("stop", [=]() { robot_->stop(); });
  }
  commandCv_.notify_one();
}

} // namespace mc_panda
