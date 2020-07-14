#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

#include <mc_rbdyn_urdf/urdf.h>

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI PandaRobotModule : public mc_rbdyn::RobotModule
{
public:
  PandaRobotModule(bool pump, bool foot, bool hand);

protected:
  // void PandaRobotModule::initConvexHull();

  std::map<std::string, std::pair<std::string, std::string>> getConvexHull(
      const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  void readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks);

  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::vector<std::map<std::string, std::vector<double>>> nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

  void init();

public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
  std::map<std::string, std::vector<double>> halfSitting;
  mc_rbdyn_urdf::Limits limits;
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"PANDA", "PANDAdefault", "PANDAhand"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PANDA")
    if(n == "PANDA" || n == "PANDAdefault")
    {
      return new mc_robots::PandaRobotModule(false, false, false);
    }
    else if(n == "PANDApump")
    {
      return new mc_robots::PandaRobotModule(true, false, false);
    }
    else if(n == "PANDAfoot")
    {
      return new mc_robots::PandaRobotModule(false, true, false);
    }
    else if(n == "PANDAhand")
    {
      return new mc_robots::PandaRobotModule(false, false, true);
    }
    else
    {
      LOG_ERROR("PANDA module cannot create an object of type " << n)
      return nullptr;
    }
  }
}
