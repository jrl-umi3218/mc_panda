#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>
#include "../devices/PandaSensor.h"

struct WaitForCollisionState : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    mc_rtc::Configuration state_conf_;

    bool sensorAvailable = false;
    const std::string sensorDeviceName = "PandaSensor";
    const std::string robname = "panda_default";

    bool collisionDetected = false;
    Eigen::Matrix<double, 7, 1> joint_contactVector_;
    Eigen::VectorXd joint_contactVector_thresholds_;
    Eigen::Matrix<double, 6, 1> cartesian_contactVector_;
    Eigen::VectorXd cartesian_contactVector_thresholds_;
    double forceThreshold_ = 10;
    mc_rbdyn::ForceSensor forceSensor;
};
