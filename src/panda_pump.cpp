#include <mc_panda/panda_pump.h>
#include <mc_panda/devices/Pump.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>

#include <RBDyn/parsers/urdf.h>
#include <mc_panda/config.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>

namespace mc_panda
{

PandaPumpRobotModule::PandaPumpRobotModule()
: mc_rbdyn::RobotModule(mc_panda::TOOLS_DESCRIPTION_PATH, "panda_pump")
{
  mc_rtc::log::success("PandaPumpRobotModule loaded with name: {} from urdf: {}", name, this->urdf_path);
  init(rbd::parsers::from_urdf_file(this->urdf_path, true));

  _default_attitude = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* Pump device */
  _devices.emplace_back(new mc_panda::Pump("panda_pump", sva::PTransformd::Identity()));

  mc_rtc::log::success("PandaPumpRobotModule uses urdf_path {}", urdf_path);
  mc_rtc::log::success("PandaPumpRobotModule uses rsdf_dir {}", rsdf_dir);
}

} // namespace mc_panda



// Export module
extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names.push_back("Panda_Tool_Pump");
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("Panda")

    if(n == "Panda_Tool_Pump")
    {
      return new mc_panda::PandaPumpRobotModule();
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No module with name {} found in PandaPumpRobotModule module", n);
    }
  }
}
