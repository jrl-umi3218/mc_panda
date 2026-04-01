#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_panda/api.h>
#include <mc_panda/config.h>

namespace mc_panda
{

struct MC_PANDA_DLLAPI PandaRobotModule : public mc_rbdyn::RobotModule
{
public:
  PandaRobotModule(bool pump, bool foot, bool hand, const std::string & urdf_path = FR3_DESCRIPTION_PATH, const std::string & rsdf_base_path = FR3_DESCRIPTION_PATH, const std::string & calib_path = FR3_DESCRIPTION_PATH);
};

} // namespace mc_panda
