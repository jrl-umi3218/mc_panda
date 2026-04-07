#include <mc_rbdyn/RobotLoader.h>
#include <mc_panda/panda.h>
#include <mc_rtc/logging.h>

#include "test_module.h"

using namespace mc_panda;

int main()
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({MODULE_DIR});

  ForAllVariants([](PandaRobots robot, Tools tool)
     {
      auto moduleName = ModuleNameFromParams(robot, tool); 
      auto rm = mc_rbdyn::RobotLoader::get_robot_module(moduleName);
      if(rm->mb.nrDof() == 0)
      {
        exit(1);
      }

      if(tool == Tools::Pump)
      {
        if(rm->devices().size() != 2)
        {
          mc_rtc::log::error_and_throw("Panda with pump tool should have 2 devices, got {}", rm->devices().size());
          if(std::find_if(rm->devices().begin(), rm->devices().end(), [](const auto & d) { return d->name() == "panda_pump"; }) == rm->devices().end())
          {
              mc_rtc::log::error_and_throw("Panda with pump tool should have a device named panda_pump");
          }
        }
      }
     });
  return 0;
}
