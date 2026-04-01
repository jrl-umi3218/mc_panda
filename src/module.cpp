#include <mc_panda/panda.h>

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>
#include <mc_panda/config.h>
#include <mc_rtc/io_utils.h>
#include <mc_rbdyn/RobotLoader.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda;
    ForAllVariants([&names](PandaRobots robot, Tools tool)
                   { names.push_back(ModuleNameFromParams(robot, tool)); });
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("Panda")
    static auto variant_factory = []()
    {
      std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> variant_factory;
      using namespace mc_panda;
      ForAllVariants(
          [&variant_factory](PandaRobots robot, Tools tool)
          {

            auto module_name = ModuleNameFromParams(robot, tool);
            variant_factory[module_name] = [=]()
            {
            using R = PandaRobots;
            using T = Tools;

            auto robot_name = RobotNameFromParams(robot, tool);

            mc_rbdyn::RobotModule * robot_rm = nullptr;

            if(robot == R::FR1)
            {
            robot_rm = new PandaRobotModule(robot_name, false, false, false, FR1_DESCRIPTION_PATH, FR1_DESCRIPTION_PATH, FR1_DESCRIPTION_PATH);
            }
            else if(robot == R::FR3)
            {
            robot_rm = new PandaRobotModule(robot_name, false, false, false, FR3_DESCRIPTION_PATH, FR3_DESCRIPTION_PATH, FR3_DESCRIPTION_PATH);
            }

            mc_rbdyn::RobotModulePtr tool_rm = nullptr;
            if(tool == T::Hand)
            {
              tool_rm = mc_rbdyn::RobotLoader::get_robot_module("Panda_Tool_Hand");
            }
            else if(tool == T::Pump)
            {
              tool_rm = mc_rbdyn::RobotLoader::get_robot_module("Panda_Tool_Pump");
            }
            else if(tool == T::Foot)
            {
              tool_rm = mc_rbdyn::RobotLoader::get_robot_module("Panda_Tool_Foot");
            }

            if(tool_rm == nullptr)
            {
              return robot_rm;
            }
            else
            {

              return new mc_rbdyn::RobotModule(
                  robot_rm->connect(
                  *tool_rm, "panda_link8", "tool_connector", "",
                  mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(sva::PTransformd::Identity())));
            }
          };
          });
      mc_rtc::log::info("mc_panda module declares the following variants:\n{}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return v.first; }));
      return variant_factory;
    }();

    // Load the desired robot variant from the factory
    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("[mc_panda] Cannot create a robot module with name '{}'", n);
      mc_rtc::log::info("Available variants are:\n{}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return fmt::format("- {}\n", v.first); }));
      return nullptr;
    }
  }
}
