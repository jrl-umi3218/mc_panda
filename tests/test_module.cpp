#include <mc_rbdyn/RobotLoader.h>

#include "test_module.h"

int main()
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({MODULE_DIR});
  for(const auto & name : {"PandaDefault", "PandaHand", "PandaFoot", "PandaPump"})
  {
    auto robot = mc_rbdyn::RobotLoader::get_robot_module(name);
    if(robot->mb.nrDof() == 0)
    {
      return 1;
    }
  }
  return 0;
}
