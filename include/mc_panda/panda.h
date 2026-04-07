#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_panda/api.h>
#include <mc_panda/config.h>

namespace mc_panda
{

enum class PandaRobots
{
  FR1, FR3
};

static std::string to_string(PandaRobots robots)
{
  using R = PandaRobots;
  switch(robots)
  {
    case R::FR1:
      return "FR1";
    case R::FR3:
      return "FR3";
    default:
      return "UnknownPanda";
  }
}

enum class Tools
{
  Default, Hand, Pump, Foot
};

static std::string to_string(Tools tools)
{
  using R = Tools;
  switch(tools)
  {
    case R::Default:
      return "Default";
    case R::Hand:
      return "Hand";
    case R::Pump:
      return "Pump";
    case R::Foot:
      return "Foot";
    default:
      return "UnknownTool";
  }
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  using R = PandaRobots;
  using T = Tools;
  for(R robot : {R::FR1, R::FR3})
  {
    for(T tool : {T::Default, T::Hand, T::Pump, T::Foot})
    {
      cb(robot, tool);
    }
  }
}

inline static std::string ModuleNameFromParams(PandaRobots robot, Tools tool, const std::string prefix = "Panda_")
{
  return prefix + to_string(robot) + "_" + to_string(tool);
}

inline static std::string RobotNameFromParams(PandaRobots robot, Tools tool)
{
  auto name = ModuleNameFromParams(robot, tool);
  std::transform(name.begin(), name.end(), name.begin(),
    [](unsigned char c){ return std::tolower(c); });
  return name;
}

struct MC_PANDA_DLLAPI PandaRobotModule : public mc_rbdyn::RobotModule
{
public:
  PandaRobotModule(const std::string & name, const std::string & urdf_path = FR3_DESCRIPTION_PATH, const std::string & rsdf_base_path = FR3_DESCRIPTION_PATH, const std::string & calib_path = FR3_DESCRIPTION_PATH);
};

mc_rbdyn::RobotModule * create(const std::string & n, const std::string & _urdf_path = "", const std::string & _rsdf_path = "", const std::string & _calib_path = "");

} // namespace mc_panda
