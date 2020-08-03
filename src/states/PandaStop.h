#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>
#include "../devices/PandaSensor.h"

struct PandaStop : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    bool sensorAvailable = false;
    const std::string sensorDeviceName = "PandaSensor";
};
