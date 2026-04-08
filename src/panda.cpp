#include <mc_panda/panda.h>
#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/Robot.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/io_utils.h>
#include <mc_rbdyn/RobotLoader.h>

#include <RBDyn/parsers/urdf.h>

namespace fs = std::filesystem;

namespace mc_panda
{

PandaRobotModule::PandaRobotModule(const std::string & _name, const std::string & _urdf_path, const std::string & _rsdf_base_path, const std::string & _calib_path)
: RobotModule(_urdf_path, _name)
{
  this->urdf_path = _urdf_path + "/urdf/panda_default.urdf";
  this->rsdf_dir = _rsdf_base_path + "/rsdf/panda_default/";
  this->calib_dir = _calib_path + "/calib";

  _real_urdf = this->urdf_path;
  mc_rtc::log::success("PandaRobotModule loaded with name: {} from urdf: {}", name, this->urdf_path);
  init(rbd::parsers::from_urdf_file(this->urdf_path, true));

  // additional joint panda limits, see https://frankaemika.github.io/docs/control_parameters.html#constants
  using bound_t = mc_rbdyn::RobotModule::accelerationBounds_t::value_type;
  bound_t torqueDerivativeUpper;
  bound_t torqueDerivativeLower;
  bound_t jerkBoundsUpper;
  bound_t jerkBoundsLower;
  bound_t accelerationBoundsUpper;
  bound_t accelerationBoundsLower;
  for(const auto & b : _bounds[0])
  {
    torqueDerivativeUpper[b.first] = std::vector<double>(b.second.size(), 1000);
    torqueDerivativeLower[b.first] = std::vector<double>(b.second.size(), -1000);
  }
  jerkBoundsLower = {{"panda_joint1", {-7500}}, {"panda_joint2", {-3750}}, {"panda_joint3", {-5000}},
                     {"panda_joint4", {-6250}}, {"panda_joint5", {-7500}}, {"panda_joint6", {-10000}},
                     {"panda_joint7", {-10000}}};
  jerkBoundsUpper = {{"panda_joint1", {7500}}, {"panda_joint2", {3750}}, {"panda_joint3", {5000}},
                     {"panda_joint4", {6250}}, {"panda_joint5", {7500}}, {"panda_joint6", {10000}},
                     {"panda_joint7", {10000}}};
  accelerationBoundsLower = {{"panda_joint1", {-15}},   {"panda_joint2", {-7.5}}, {"panda_joint3", {-10}},
                             {"panda_joint4", {-12.5}}, {"panda_joint5", {-15}},  {"panda_joint6", {-20}},
                             {"panda_joint7", {-20}}};
  accelerationBoundsUpper = {{"panda_joint1", {15}},   {"panda_joint2", {7.5}}, {"panda_joint3", {10}},
                             {"panda_joint4", {12.5}}, {"panda_joint5", {15}},  {"panda_joint6", {20}},
                             {"panda_joint7", {20}}};
  _torqueDerivativeBounds.push_back(torqueDerivativeLower);
  _torqueDerivativeBounds.push_back(torqueDerivativeUpper);
  _jerkBounds.push_back(jerkBoundsLower);
  _jerkBounds.push_back(jerkBoundsUpper);
  _accelerationBounds.push_back(accelerationBoundsLower);
  _accelerationBounds.push_back(accelerationBoundsUpper);

  rsdf_dir = path + "/rsdf/" + name + "/";
  calib_dir = path + "/calib";

  _bodySensors.clear();

  _stance["panda_joint1"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_joint2"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_joint3"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_joint4"] = {mc_rtc::constants::toRad(-120)};
  _stance["panda_joint5"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_joint6"] = {mc_rtc::constants::toRad(120)};
  _stance["panda_joint7"] = {mc_rtc::constants::toRad(0)};

  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("LeftHandForceSensor", "panda_link7",
                            sva::PTransformd(mc_rbdyn::rpyToMat(3.14, 0.0, 0.0), Eigen::Vector3d(0, 0, -0.04435))));

  // Add convex shapes from sch files to _convexHull if they exist (they do for FR1)
  // NOTE that these collision shapes cannot be used directly on the real robot as the embedded controller
  // independently checks for collision based on capsules.
  auto convexPath = path + "/convex/panda_default";
  for(const auto & b : mb.bodies())
  {
    auto ch = fs::path{convexPath} / (b.name() + "-ch.txt");
    if(fs::exists(ch))
    {
      auto colName = "convex_" + b.name();
      _convexHull[colName] = {b.name(), ch.string()};
      _collisionTransforms[colName] = sva::PTransformd::Identity();
    }
  }

  // By default we use very conservative self-collision shapes (capsules) defined in the urdf
  // These match the ones used internally by the robot such that we do not trigger
  // the self_collision_constraint_violation check.
  const double i = 0.01;
  const double s = 0.005;
  const double d = 0.;
  // clang-format off
  _minimalSelfCollisions =
  {
    {"panda_link0*", "panda_link5*", i, s, d},
    {"panda_link1*", "panda_link5*", i, s, d},
    {"panda_link2*", "panda_link5*", i, s, d},
    {"panda_link3*", "panda_link5*", i, s, d},
    {"panda_link0*", "panda_link6*", i, s, d},
    {"panda_link1*", "panda_link6*", i, s, d},
    {"panda_link2*", "panda_link6*", i, s, d},
    {"panda_link0*", "panda_link7*", i, s, d},
    {"panda_link1*", "panda_link7*", i, s, d},
    {"panda_link2*", "panda_link7*", i, s, d},
    {"panda_link3*", "panda_link7*", i, s, d}
  };
  // clang-format on

  /* Additional self collisions */
  // if(pump)
  // {
  //   // FIXME No pump convex ATM
  //   //_commonSelfCollisions.push_back({"panda_link0", "pump", i, s, d)};
  //   //_commonSelfCollisions.push_back({"panda_link1", "pump", i, s, d)};
  //   //_commonSelfCollisions.push_back({"panda_link2", "pump", i, s, d)};
  //   //_commonSelfCollisions.push_back({"panda_link3", "pump", i, s, d)};
  // }
  // if(foot)
  // {
  //   _commonSelfCollisions.push_back({"panda_link0", "foot", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link1", "foot", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link2", "foot", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link3", "foot", i, s, d});
  // }
  // if(hand)
  // {
  //   _commonSelfCollisions.push_back({"panda_link0", "hand", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link1", "hand", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link2", "hand", i, s, d});
  //   _commonSelfCollisions.push_back({"panda_link3", "hand", i, s, d});
  // }

  _commonSelfCollisions = _minimalSelfCollisions;

  _ref_joint_order = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                      "panda_joint5", "panda_joint6", "panda_joint7"};

  _default_attitude = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // NOTE: this is a joint-space sensor and not attached to a specific Cartesian link with a certain transformation
  _devices.emplace_back(new mc_panda::Robot());

  mc_rtc::log::success("PandaRobotModule uses urdf_path {}", urdf_path);
  mc_rtc::log::success("PandaRobotModule uses rsdf_dir {}", rsdf_dir);
}



mc_rbdyn::RobotModule * create(const std::string & n, const std::string & _urdf_path, const std::string & _rsdf_path, const std::string & _calib_path)
{
  using namespace mc_panda;
  using R = PandaRobots;
  using T = Tools;

  mc_rbdyn::RobotModule * result = nullptr;
  bool found = false;

  ForAllVariants([&](PandaRobots robot, Tools tool)
  {
    if(found) return; // already found, skip

    auto module_name = ModuleNameFromParams(robot, tool);
    if(module_name == n)
    {
      found = true;
      auto robot_name = RobotNameFromParams(robot, tool);

      auto urdf_path = _urdf_path.empty() ? (robot == R::FR1 ? FR1_DESCRIPTION_PATH : FR3_DESCRIPTION_PATH) : _urdf_path;
      auto rsdf_path = _rsdf_path.empty() ? (robot == R::FR1 ? FR1_DESCRIPTION_PATH : FR3_DESCRIPTION_PATH) : _rsdf_path;
      auto calib_path = _calib_path.empty() ? (robot == R::FR1 ? FR1_DESCRIPTION_PATH : FR3_DESCRIPTION_PATH) : _calib_path;

      mc_rbdyn::RobotModule * robot_rm = new PandaRobotModule(robot_name, urdf_path, rsdf_path, calib_path);

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
        result = robot_rm;
      }
      else
      {
        result = new mc_rbdyn::RobotModule(
          robot_rm->connect(
            *tool_rm, "panda_link8", "tool_connector", "",
            mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(sva::PTransformd::Identity())));
      }
    }
  });

  if(result)
  {
    return result;
  }
  else
  {
    mc_rtc::log::error("[mc_panda] Cannot create a robot module with name '{}'", n);
    // Optionally, print available variants:
    std::string variants;
    ForAllVariants([&](PandaRobots robot, Tools tool)
    {
      variants += "- " + ModuleNameFromParams(robot, tool) + "\n";
    });
    mc_rtc::log::info("Available variants are:\n{}", variants);
    return nullptr;
  }
}

} // namespace mc_panda


