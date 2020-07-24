#include "WaitForCollisionState.h"
#include <mc_tasks/MetaTaskLoader.h>

void WaitForCollisionState::configure(const mc_rtc::Configuration & config)
{
  joint_contactVector_ = Eigen::Matrix<double, 7, 1> ::Zero();
  joint_contactVector_thresholds_ = Eigen::Matrix<double, 7, 1> ::Ones() * std::numeric_limits<double>::infinity();
  
  cartesian_contactVector_ = Eigen::Matrix<double, 6, 1> ::Zero();
  cartesian_contactVector_thresholds_ = Eigen::Matrix<double, 6, 1> ::Ones()*  std::numeric_limits<double>::infinity();

  forceThreshold_ = std::numeric_limits<double>::infinity();

  state_conf_.load(config);
  if(state_conf_.has("jointContactThresholds")){
    mc_rtc::log::success("in configure method loading the config, found jointContactThresholds");
    joint_contactVector_thresholds_ = state_conf_("jointContactThresholds");
    if(joint_contactVector_thresholds_.size() != 7)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the jointContactThresholds does not contain 7 elements... {}", joint_contactVector_thresholds_);
    }
    else{
      mc_rtc::log::info("use jointContactThresholds {}", joint_contactVector_thresholds_.transpose());
    }
  }
  else
  {
    mc_rtc::log::info("cannot find jointContactThresholds in config, use: {}", joint_contactVector_thresholds_.transpose());
  }
  if(state_conf_.has("cartesianContactThresholds")){
    mc_rtc::log::success("in configure method loading the config, found cartesianContactThresholds");
    cartesian_contactVector_thresholds_ = state_conf_("cartesianContactThresholds");
    if(cartesian_contactVector_thresholds_.size() != 6)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the cartesianContactThresholds does not contain 6 elements... {}", cartesian_contactVector_thresholds_);
    }
    else{
      mc_rtc::log::info("use cartesianContactThresholds {}", cartesian_contactVector_thresholds_.transpose());
    }
  }
  else
  {
    mc_rtc::log::info("cannot find cartesianContactThresholds in config, use: {}", cartesian_contactVector_thresholds_.transpose());
  }
  if(state_conf_.has("forceThreshold")){
    mc_rtc::log::success("in configure method loading the config, found forceThreshold");
    forceThreshold_ = state_conf_("forceThreshold");
    if(forceThreshold_ < 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the forceThreshold is not positive... {}", forceThreshold_);
    }
    else{
      mc_rtc::log::info("use forceThreshold {}", forceThreshold_);
    }
  }
  else
  {
    mc_rtc::log::info("cannot find forceThreshold in config, use: {}", forceThreshold_);
  }

  joint_contactVector_thresholds_log_ = Eigen::Matrix<double, 14, 1> ::Zero();
  joint_contactVector_thresholds_log_ << joint_contactVector_thresholds_, -1*joint_contactVector_thresholds_;
  cartesian_contactVector_thresholds_log_ = Eigen::Matrix<double, 12, 1> ::Zero();
  cartesian_contactVector_thresholds_log_ << cartesian_contactVector_thresholds_, -1*cartesian_contactVector_thresholds_; 
}

void WaitForCollisionState::start(mc_control::fsm::Controller & ctl_)
{
  addToLogger(ctl_.logger());

  if(ctl_.robot().hasDevice<mc_panda::PandaSensor>(sensorDeviceName))
  {
    sensorAvailable = true;
    mc_rtc::log::info("RobotModule has a PandaSensor named {}", sensorDeviceName);
  }
  else{
    mc_rtc::log::warning("RobotModule does not have a PandaSensor named {}", sensorDeviceName);
    mc_rtc::log::warning("PandaSensor functionality will not be available");
  }

  forceSensor = ctl_.robot().forceSensor("LeftHandForceSensor");

  mc_rtc::log::success("WaitForCollisionState state start done");
}

bool WaitForCollisionState::run(mc_control::fsm::Controller & ctl_)
{
  if(collisionDetected){
    output("OK");
    return true;
  }

  if(sensorAvailable){
    joint_contactVector_ = ctl_.robot().device<mc_panda::PandaSensor>(sensorDeviceName).get_tau_ext_hat_filtered();
    for(int i=0; i<7; i++){
      if(fabs(joint_contactVector_(i)) > joint_contactVector_thresholds_(i))
      {
        mc_rtc::log::info("WaitForCollisionState detected a joint-space collision for index {}: abs({}) > {}", i, joint_contactVector_(i), joint_contactVector_thresholds_(i));
        mc_rtc::log::info("WaitForCollisionState joint_contactVector_: {}", joint_contactVector_.transpose());
        mc_rtc::log::info("Completed WaitForCollisionState");
        collisionDetected = true;
        output("OK");
        return true;
      }
    }
    cartesian_contactVector_ = ctl_.robot().device<mc_panda::PandaSensor>(sensorDeviceName).get_K_F_ext_hat_K();
    for(int i=0; i<6; i++){
      if(fabs(cartesian_contactVector_(i)) > cartesian_contactVector_thresholds_(i))
      {
        mc_rtc::log::info("WaitForCollisionState detected a cartesian-space collision for index {}: abs({}) > {}", i, cartesian_contactVector_(i), cartesian_contactVector_thresholds_(i));
        mc_rtc::log::info("WaitForCollisionState cartesian_contactVector_: {}", cartesian_contactVector_.transpose());
        mc_rtc::log::info("Completed WaitForCollisionState");
        collisionDetected = true;
        output("OK");
        return true;
      }
    }
  }
  if(fabs(forceSensor.wrench().force().z()) > forceThreshold_)
  {
      mc_rtc::log::info("WaitForCollisionState detected a force().z() collision: abs({}) > {}", forceSensor.wrench().force().z(), forceThreshold_);
      mc_rtc::log::info("Completed WaitForCollisionState");
      collisionDetected = true;
      output("OK");
      return true;
  }
  // mc_rtc::log::info("WaitForCollisionState did not detect a collision, current force is: {}", forceSensor.wrench().force().transpose());
  return false;
}

void WaitForCollisionState::teardown(mc_control::fsm::Controller & ctl_)
{
  removeFromLogger(ctl_.logger());

  mc_rtc::log::info("WaitForCollisionState teardown done");
}

void WaitForCollisionState::addToLogger(mc_rtc::Logger & logger)
{
  std::string logname = "WaitForCollisionState_";
  logger.addLogEntry(logname + "tauexthatfilteredThresholds", 
    [this]() {
      return (Eigen::VectorXd) joint_contactVector_thresholds_log_; 
    }
  );
  logger.addLogEntry(logname + "OFexthatKThresholds", 
    [this]() {
      return (Eigen::VectorXd) cartesian_contactVector_thresholds_log_; 
    }
  );
  mc_rtc::log::info("PandaSensor device started to log data");
}

void WaitForCollisionState::removeFromLogger(mc_rtc::Logger & logger)
{
  std::string logname = "WaitForCollisionState_";
  logger.removeLogEntry(logname + "tauexthatfilteredThresholds");
  logger.removeLogEntry(logname + "OFexthatKThresholds");
}

EXPORT_SINGLE_STATE("WaitForCollisionState", WaitForCollisionState)
