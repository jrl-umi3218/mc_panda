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
  Default, Hand, Pump, Foot, Mukca, PandaToPandaCalib
};

static std::string to_string(Tools tools)
{
  using T = Tools;
  switch(tools)
  {
    case T::Default:
      return "Default";
    case T::Hand:
      return "Hand";
    case T::Pump:
      return "Pump";
    case T::Foot:
      return "Foot";
    case T::Mukca:
      return "Mukca";
    case T::PandaToPandaCalib:
      return "PandaToPandaCalib";
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
    for(T tool : {T::Default, T::Hand, T::Pump, T::Foot, T::Mukca, T::PandaToPandaCalib})
    {
      cb(robot, tool);
    }
  }
}

inline static std::string ModuleNameFromParams(PandaRobots robot, Tools tool, const std::string prefix = "Panda_")
{
  return prefix + to_string(robot) + "_" + to_string(tool);
}

inline static std::string to_lower_case(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(),
    [](unsigned char c){ return std::tolower(c); });
  return name;
}

inline static std::string RobotNameFromParams(PandaRobots robot, Tools tool)
{
  auto name = ModuleNameFromParams(robot, tool);
  to_lower_case(name);
  return name;
}

struct PathsConfiguration
{
  std::string urdf_base_path = "";
  std::string convex_base_path = "";
  std::string rsdf_base_path = "";
  std::string calib_base_path = "";
};

static auto FR3DefaultPaths = PathsConfiguration{FR3_DESCRIPTION_PATH, FR3_DESCRIPTION_PATH, FR3_DESCRIPTION_PATH, FR3_DESCRIPTION_PATH};
static auto FR1DefaultPaths = PathsConfiguration{FR1_DESCRIPTION_PATH, FR1_DESCRIPTION_PATH, FR1_DESCRIPTION_PATH, FR1_DESCRIPTION_PATH};

struct MC_PANDA_DLLAPI PandaRobotModule : public mc_rbdyn::RobotModule
{
public:
  PandaRobotModule(const std::string & name, const PathsConfiguration & pathsConfig);
};

mc_rbdyn::RobotModule * create(const std::string & n, const std::optional<PathsConfiguration> & pathsConfig = std::nullopt);

} // namespace mc_panda
