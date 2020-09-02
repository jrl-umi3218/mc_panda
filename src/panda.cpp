#include "panda.h"
#include "devices/PandaDevice.h"
#include "devices/Pump.h"

#include "config.h"

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>

#include <RBDyn/parsers/urdf.h>

namespace mc_robots
{

inline static std::string pandaVariant(bool pump, bool foot, bool hand)
{
  if(pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_pump'");
    return "panda_pump";
  }
  if(!pump && foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_foot'");
    return "panda_foot";
  }
  if(!pump && !foot && hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_hand'");
    return "panda_hand";
  }
  if(!pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_default'");
    return "panda_default";
  }
  mc_rtc::log::error("PandaRobotModule does not provide this panda variant...");
  return "";
}

PandaRobotModule::PandaRobotModule(bool pump, bool foot, bool hand)
: RobotModule(PANDA_DESCRIPTION_PATH, pandaVariant(pump, foot, hand))
{
  mc_rtc::log::success("PandaRobotModule loaded with name: {}", name);
  urdf_path = path + "/" + name + ".urdf";
  _real_urdf = urdf_path;
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  // additional joint panda limits, see https://frankaemika.github.io/docs/control_parameters.html#constants
  std::map<std::string, std::vector<double>> torqueDerivativeUpper;
  std::map<std::string, std::vector<double>> torqueDerivativeLower;
  std::map<std::string, std::vector<double>> accelerationBoundsUpper;
  std::map<std::string, std::vector<double>> accelerationBoundsLower;
  for(const auto & b : _bounds[0])
  {
    torqueDerivativeUpper[b.first] = std::vector<double>(b.second.size(), 1000);
    torqueDerivativeLower[b.first] = std::vector<double>(b.second.size(), -1000);
  }
  accelerationBoundsLower = {{"panda_jointA1", {-15}},   {"panda_jointA2", {-7.5}}, {"panda_jointA3", {-10}},
                             {"panda_jointA4", {-12.5}}, {"panda_jointA5", {-15}},  {"panda_jointA6", {-20}},
                             {"panda_jointA7", {-20}}};
  accelerationBoundsUpper = {{"panda_jointA1", {15}},   {"panda_jointA2", {7.5}}, {"panda_jointA3", {10}},
                             {"panda_jointA4", {12.5}}, {"panda_jointA5", {15}},  {"panda_jointA6", {20}},
                             {"panda_jointA7", {20}}};
  _torqueDerivativeBounds.push_back(torqueDerivativeLower);
  _torqueDerivativeBounds.push_back(torqueDerivativeUpper);
  _accelerationBounds.push_back(accelerationBoundsLower);
  _accelerationBounds.push_back(accelerationBoundsUpper);

  rsdf_dir = path + "/rsdf/" + name + "/";
  calib_dir = path + "/calib";

  _bodySensors.clear();

  _stance["panda_jointA1"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_jointA2"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_jointA3"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_jointA4"] = {mc_rtc::constants::toRad(-120)};
  _stance["panda_jointA5"] = {mc_rtc::constants::toRad(0)};
  _stance["panda_jointA6"] = {mc_rtc::constants::toRad(120)};
  _stance["panda_jointA7"] = {mc_rtc::constants::toRad(0)};

  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("LeftHandForceSensor", "panda_linkA7",
                            sva::PTransformd(mc_rbdyn::rpyToMat(3.14, 0.0, 0.0), Eigen::Vector3d(0, 0, -0.04435))));

  if(hand)
  {
    _convexHull["panda_hand"] = {"panda_hand", path + "/convex/panda_hand/panda_hand-ch.txt"};
  }
  if(foot)
  {
    _convexHull["panda_foot"] = {"panda_foot", path + "/convex/panda_foot/panda_foot-ch.txt"};
  }
  if(pump)
  {
    // FIXME No pump convex ATM
    //_convexHull["pump"] = {"panda_pump", path + "/convex/panda_pump/panda_pump-ch.txt"};
  }

  const double i = 0.015; // 0.01;
  const double s = 0.0075; // 0.005;
  const double d = 0.;
  _minimalSelfCollisions = {mc_rbdyn::Collision("panda_linkA0*", "panda_linkA5*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA1*", "panda_linkA5*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA2*", "panda_linkA5*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA3*", "panda_linkA5*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA0*", "panda_linkA6*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA1*", "panda_linkA6*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA2*", "panda_linkA6*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA3*", "panda_linkA6*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA0*", "panda_linkA7*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA1*", "panda_linkA7*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA2*", "panda_linkA7*", i, s, d),
                            mc_rbdyn::Collision("panda_linkA3*", "panda_linkA7*", i, s, d),
                            // FIXME Is this last one needed?
                            mc_rbdyn::Collision("panda_linkA5*", "panda_linkA7*", i, s, d)};

  /* Additional self collisions */
  if(pump)
  {
    // FIXME No pump convex ATM
    //_commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "pump", i, s, d));
    //_commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "pump", i, s, d));
    //_commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "pump", i, s, d));
    //_commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "pump", i, s, d));
  }
  if(foot)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "foot", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "foot", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "foot", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "foot", i, s, d));
  }
  if(hand)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "hand", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "hand", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "hand", i, s, d));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "hand", i, s, d));
  }

  _commonSelfCollisions = _minimalSelfCollisions;

  _ref_joint_order = {"panda_jointA1", "panda_jointA2", "panda_jointA3", "panda_jointA4",
                      "panda_jointA5", "panda_jointA6", "panda_jointA7"};

  _default_attitude = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // NOTE: this is a joint-space sensor and not attached to a specific Cartesian link with a certain transformation
  _devices.emplace_back(new mc_panda::PandaDevice());
  if(pump)
  {
    /* Pump device */
    _devices.emplace_back(new mc_panda::Pump("Pump", "panda_linkA8", sva::PTransformd::Identity()));
  }

  /* Grippers */
  if(hand)
  {
    // Module wide gripper configuration
    _gripperSafety = {0.15, 1.0};
    _grippers = {{"l_gripper", {"L_HAND_J0", "L_HAND_J1"}, false}};
  }

  mc_rtc::log::success("PandaRobotModule uses urdf_path {}", urdf_path);
  mc_rtc::log::success("PandaRobotModule uses rsdf_dir {}", rsdf_dir);
}

} // namespace mc_robots
