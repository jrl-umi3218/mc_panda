#include "WaitForCollisionState.h"
#include <mc_tasks/MetaTaskLoader.h>

void WaitForCollisionState::configure(const mc_rtc::Configuration & config)
{
  joint_contactVector_ = Eigen::Matrix<double, 7, 1> ::Zero();
  joint_contactVector_thresholds_ = Eigen::Matrix<double, 7, 1> ::Ones();
  joint_contactVector_thresholds_ = joint_contactVector_thresholds_ * std::numeric_limits<double>::infinity();

  cartesian_contactVector_ = Eigen::Matrix<double, 6, 1> ::Zero();
  cartesian_contactVector_thresholds_ = Eigen::Matrix<double, 6, 1> ::Ones();
  cartesian_contactVector_thresholds_ = cartesian_contactVector_thresholds_ * std::numeric_limits<double>::infinity();

  state_conf_.load(config);
  if(state_conf_.has("jointContactThresholds")){
    mc_rtc::log::success("in configure method loading the config, found jointContactThresholds");
    joint_contactVector_thresholds_ = state_conf_("jointContactThresholds");
    if(joint_contactVector_thresholds_.size() != 7)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the jointContactThresholds does not contain 7 elements... {}", joint_contactVector_thresholds_);
    }
    else{
      mc_rtc::log::info("use jointContactThresholds {}", joint_contactVector_thresholds_);
    }
  }
  if(state_conf_.has("cartesianContactThresholds")){
    mc_rtc::log::success("in configure method loading the config, found cartesianContactThresholds");
    cartesian_contactVector_thresholds_ = state_conf_("cartesianContactThresholds");
    if(cartesian_contactVector_thresholds_.size() != 6)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the cartesianContactThresholds does not contain 6 elements... {}", cartesian_contactVector_thresholds_);
    }
    else{
      mc_rtc::log::info("use cartesianContactThresholds {}", cartesian_contactVector_thresholds_);
    }
  }
  if(state_conf_.has("forceThreshold")){
    mc_rtc::log::success("in configure method loading the config, found forceThreshold");
    forceThreshold_ = state_conf_("forceThreshold");
    if(forceThreshold_ > 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("the forceThreshold is not positive... {}", forceThreshold_);
    }
    else{
      mc_rtc::log::info("use forceThreshold {}", forceThreshold_);
    }
  }
}

void WaitForCollisionState::start(mc_control::fsm::Controller & ctl_)
{
  if(ctl_.robot(robname).hasDevice<mc_panda::PandaSensor>(sensorDeviceName))
  {
    sensorAvailable = true;
    mc_rtc::log::info("RobotModule has a PandaSensor named {}", sensorDeviceName);
  }
  else{
    mc_rtc::log::warning("RobotModule does not have a PandaSensor named {}", sensorDeviceName);
    mc_rtc::log::warning("PandaSensor functionality will not be available");
  }

  forceSensor = ctl_.robot(robname).forceSensor("LeftHandForceSensor");

  mc_rtc::log::success("WaitForCollisionState state start done");
}

bool WaitForCollisionState::run(mc_control::fsm::Controller & ctl_)
{
  if(collisionDetected){
    output("OK");
    return true;
  }

  if(sensorAvailable){
    for(int i=0; i<7; i++){
      joint_contactVector_ = ctl_.robot(robname).device<mc_panda::PandaSensor>(sensorDeviceName).get_joint_contact();
      if(joint_contactVector_(i) > joint_contactVector_thresholds_(i))
      {
        mc_rtc::log::info("WaitForCollisionState detected a joint-space collision: {}", joint_contactVector_thresholds_);
        mc_rtc::log::info("Completed WaitForCollisionState");
        collisionDetected = true;
        output("OK");
        return true;
      }
    }
    for(int i=0; i<6; i++){
      cartesian_contactVector_ = ctl_.robot(robname).device<mc_panda::PandaSensor>(sensorDeviceName).get_cartesian_contact();
      if(cartesian_contactVector_(i) > cartesian_contactVector_thresholds_(i))
      {
        mc_rtc::log::info("WaitForCollisionState detected a cartesian-space collision: {}", cartesian_contactVector_);
        mc_rtc::log::info("Completed WaitForCollisionState");
        collisionDetected = true;
        output("OK");
        return true;
      }
    }
  }
  sva::ForceVecd wrench = forceSensor.wrench();
  if(wrench.force().z() < -forceThreshold_) //TODO or > ??
  {
      mc_rtc::log::info("WaitForCollisionState detected a force().z() collision: {}", wrench.force().z());
      mc_rtc::log::info("Completed WaitForCollisionState");
      collisionDetected = true;
      output("OK");
      return true;
  }
  // mc_rtc::log::info("WaitForCollisionState did not detect a collision, current force is: {}", wrench.force().transpose());
  return false;
}

void WaitForCollisionState::teardown(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("WaitForCollisionState teardown done");
}

EXPORT_SINGLE_STATE("WaitForCollisionState", WaitForCollisionState)
