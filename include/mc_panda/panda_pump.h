#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_panda/api.h>
#include <mc_panda/config.h>

namespace mc_panda
{

struct MC_PANDA_DLLAPI PandaPumpRobotModule : public mc_rbdyn::RobotModule
{
public:
  PandaPumpRobotModule();
};

} // namespace mc_panda
