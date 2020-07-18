#include "panda.h"
#include "devices/Pump.h"

#include "config.h"

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

#include <mc_rbdyn/rpy_utils.h>

#include <RBDyn/parsers/urdf.h>

namespace mc_robots
{

inline static std::string pandaVariant(bool pump, bool foot, bool hand)
{
  if(pump)
  {
    return "panda_arm_pump";
  }
  if(foot)
  {
    return "panda_arm_foot";
  }
  if(hand)
  {
    return "panda_arm_hand";
  }
  return "panda_arm_default";
}

inline double deg2rad(double theta)
{
  return theta * M_PI / 180;
}

PandaRobotModule::PandaRobotModule(bool pump, bool foot, bool hand) : RobotModule(PANDA_DESCRIPTION_PATH, pandaVariant(pump, foot, hand))
{
  urdf_path = path + "/" + name + ".urdf";
  _real_urdf = urdf_path;
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  std::map<std::string, std::vector<double>> torqueDerivativeUpper;
  std::map<std::string, std::vector<double>> torqueDerivativeLower;
  for(const auto & b : _bounds[0])
  {
    torqueDerivativeUpper[b.first] = std::vector<double>(b.second.size(), 1000);
    torqueDerivativeLower[b.first] = std::vector<double>(b.second.size(), -1000);
  }
  _bounds.push_back(torqueDerivativeLower);
  _bounds.push_back(torqueDerivativeUpper);

  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";

  _bodySensors.clear();

  /* Default posture joint values in degrees */
  _stance["panda_jointA1"] = {deg2rad(0)};
  _stance["panda_jointA2"] = {deg2rad(0)};
  _stance["panda_jointA3"] = {deg2rad(0)};
  _stance["panda_jointA4"] = {deg2rad(-120)};
  _stance["panda_jointA5"] = {deg2rad(0)};
  _stance["panda_jointA6"] = {deg2rad(120)};
  _stance["panda_jointA7"] = {deg2rad(0)};

  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "l_wrist", sva::PTransformd(mc_rbdyn::rpyToMat(3.14,0.0,0.0), Eigen::Vector3d(0, 0, -0.04435))));

  for(size_t i = 0; i < 8; ++i)
  {
    std::string link = "panda_linkA" + std::to_string(i);
    std::string cpath = path + "/convex/panda_default/panda_link" + std::to_string(i) + "-ch.txt";
    _convexHull[link] = {link, cpath};
  }
  if(hand)
  {
    _convexHull["panda_hand"] = {"panda_hand", path + "/convex/panda_hand/panda_hand-ch.txt"};
  }
  if(foot)
  {
    _convexHull["panda_foot"] = {"panda_foot", path + "/convex/panda_foot/panda_foot-ch.txt"};
  }
  // FIXME Do we have a pump convex hull?

  _minimalSelfCollisions = {mc_rbdyn::Collision("panda_linkA0", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA0", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA0", "panda_linkA7", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA7", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA7", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA7", 0.06, 0.02, 0.),
                            // FIXME Is this last one needed?
                            mc_rbdyn::Collision("panda_linkA5", "panda_linkA7", 0.06, 0.02, 0.)
                            };
  /* Additional self collisions */
  if(pump)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "pump", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "pump", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "pump", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "pump", 0.06, 0.02, 0.));
  }
  if(foot)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "foot", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "foot", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "foot", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "foot", 0.06, 0.02, 0.));
  }
  if(hand)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA0", "hand", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA1", "hand", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA2", "hand", 0.06, 0.02, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("panda_linkA3", "hand", 0.06, 0.02, 0.));
  }

  _commonSelfCollisions = _minimalSelfCollisions;

  _ref_joint_order = {"panda_jointA1", "panda_jointA2", "panda_jointA3", "panda_jointA4", "panda_jointA5", "panda_jointA6", "panda_jointA7"};

  _default_attitude = {{}};

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
}

} // namespace mc_robots
