#include <mc_rbdyn/RobotLoader.h>

#include "test_module.h"

int main()
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({MODULE_DIR});
  auto robot = mc_rbdyn::RobotLoader::get_robot_module(MODULE_NAME);
  std::cout << MODULE_NAME << " has " << robot->mb.nrDof() << " dof\n";
  return 0;
}