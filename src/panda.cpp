// #ifndef PANDA_DESCRIPTION_PATH
// #  error "PANDA_DESCRIPTION_PATH must be defined to build this RobotModule"
// #endif

#include "panda.h"

#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>

#include <fstream>

#include <mc_rbdyn/rpy_utils.h>

#include "config.h"

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

PANDARobotModule::PANDARobotModule() : RobotModule(PANDA_DESCRIPTION_PATH, "panda")
{
  // init(rbd::parsers::from_urdf_file(urdf_path, false));
  // initConvexHull();
  // std::vector<double> default_q = {0., 0., 0., -2., 0.,  2., 0};
  // const auto & rjo = ref_joint_order();
  // for(size_t i = 0; i < rjo.size(); ++i)
  // {
  //   _stance[rjo[i]] = {default_q[i]};
  // }

  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";

  virtualLinks.clear();

  gripperLinks.clear();

  _bodySensors.clear();

  halfSitting["panda_jointA1"] = {0};
  halfSitting["panda_jointA2"] = {0};
  halfSitting["panda_jointA3"] = {0};
  halfSitting["panda_jointA4"] = {-120};
  halfSitting["panda_jointA5"] = {0};
  halfSitting["panda_jointA6"] = {120};
  halfSitting["panda_jointA7"] = {0};

  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "l_wrist", sva::PTransformd(mc_rbdyn::rpyToMat(3.14,0.0,0.0), Eigen::Vector3d(0, 0, -0.04435))));

  _minimalSelfCollisions = {mc_rbdyn::Collision("panda_linkA0", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA5", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA0", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA6", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA7", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA0", "panda_linkA8", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA1", "panda_linkA8", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA2", "panda_linkA8", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA3", "panda_linkA8", 0.06, 0.02, 0.),
                            mc_rbdyn::Collision("panda_linkA5", "panda_linkA7", 0.06, 0.02, 0.) //TODO do we need the last self-collision?
                            };

  _commonSelfCollisions = _minimalSelfCollisions;
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("...", "...", 0.02, 0.01, 0.));

  _grippers = {}; //{{"l_gripper", {"L_HAND_J0", "L_HAND_J1"}, false}};

  _ref_joint_order = {"panda_jointA1", "panda_jointA2", "panda_jointA3", "panda_jointA4", "panda_jointA5", "panda_jointA6", "panda_jointA7"};

  _default_attitude = {{}}; //{{1., 0., 0., 0., 0., 0., 0.747187}};
}

// void PANDARobotModule::initConvexHull() 
// {
//   std::string convexPath = path + "/convex/" + name + "/";
//   bfs::path p(convexPath);
//   if(bfs::exists(p) && bfs::is_directory(p))
//   {
//     std::vector<bfs::path> files;
//     std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
//     for(const bfs::path & file : files)
//     {
//       size_t off = file.filename().string().rfind("-ch.txt");
//       if(off != std::string::npos)
//       {
//         std::string name = file.filename().string();
//         name.replace(off, 7, "");
//         _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
//       }
//     }
//   }
// }

std::map<std::string, std::pair<std::string, std::string>> PANDARobotModule::getConvexHull(
    const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/";
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

void PANDARobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  urdf_path = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), true, filteredLinks);
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    limits = res.limits;

    _visual = res.visual;
    _collisionTransforms = res.collision_tf;
  }
  else
  {
    LOG_ERROR("Could not open PANDA model at " << urdf_path)
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to open PANDA model")
  }
}

void PANDARobotModule::init()
{
  _springs.springsBodies = {}; //{"l_ankle", "r_ankle"};
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
}

std::map<std::string, std::vector<double>> PANDARobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & j : mb.joints())
  {
    if(halfSitting.count(j.name()))
    {
      res[j.name()] = halfSitting.at(j.name());
      for(auto & ji : res[j.name()])
      {
        ji = M_PI * ji / 180;
      }
    }
    else if(j.name() != "Root" && j.dof() > 0)
    {
      LOG_WARNING("Joint " << j.name() << " has " << j.dof() << " dof, but is not part of half sitting posture.");
    }
  }
  return res;
}

std::vector<std::map<std::string, std::vector<double>>> PANDARobotModule::nominalBounds(
    const mc_rbdyn_urdf::Limits & limits) const
{
  std::vector<std::map<std::string, std::vector<double>>> res(0);
  res.push_back(limits.lower);
  res.push_back(limits.upper);
  {
    auto mvelocity = limits.velocity;
    for(auto & mv : mvelocity)
    {
      for(auto & mvi : mv.second)
      {
        mvi = -mvi;
      }
    }
    res.push_back(mvelocity);
  }
  res.push_back(limits.velocity);
  {
    auto mtorque = limits.torque;
    for(auto & mt : mtorque)
    {
      for(auto & mti : mt.second)
      {
        mti = -mti;
      }
    }
    res.push_back(mtorque);
  }
  res.push_back(limits.torque);
  return res;
}

std::map<std::string, std::pair<std::string, std::string>> PANDARobotModule::stdCollisionsFiles(
    const rbd::MultiBody & mb) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & b : mb.bodies())
  {
    // Filter out virtual links without convex files
    if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
    {
      res[b.name()] = {b.name(), boost::algorithm::replace_first_copy(b.name(), "_LINK", "")};
    }
  }

  auto addBody = [&res](const std::string & body, const std::string & file) { res[body] = {body, file}; };
  addBody("body", "WAIST_LINK");
  addBody("torso", "CHEST_Y");

  addBody("R_HIP_Y_LINK", "HIP_Y");
  addBody("R_HIP_R_LINK", "CHEST_P");
  addBody("R_ANKLE_P_LINK", "L_ANKLE_P");
  addBody("r_ankle", "R_FOOT");

  addBody("L_HIP_Y_LINK", "HIP_Y");
  addBody("L_HIP_R_LINK", "CHEST_P");
  addBody("l_ankle", "L_FOOT");

  addBody("CHEST_P_LINK", "CHEST");

  addBody("R_SHOULDER_Y_LINK", "SHOULDER_Y");
  addBody("R_ELBOW_P_LINK", "ELBOW_P");
  addBody("R_WRIST_P_LINK", "WRIST_P");
  addBody("r_wrist", "R_WRIST_R");

  auto finger = [&addBody](const std::string & prefix) {
    addBody(prefix + "_HAND_J0_LINK", prefix + "_THUMB");
    addBody(prefix + "_HAND_J1_LINK", prefix + "_F1");
    for(unsigned int i = 2; i < 6; ++i)
    {
      std::stringstream key1;
      key1 << prefix << "_F" << i << "2_LINK";
      std::stringstream key2;
      key2 << prefix << "_F" << i << "3_LINK";
      addBody(key1.str(), "F2");
      addBody(key2.str(), "F3");
    }
  };
  finger("R");
  finger("L");

  addBody("L_SHOULDER_Y_LINK", "SHOULDER_Y");
  addBody("L_ELBOW_P_LINK", "ELBOW_P");
  addBody("L_WRIST_P_LINK", "WRIST_P");
  addBody("l_wrist", "L_WRIST_R");

  auto addWristSubConvex = [&res](const std::string & prefix) {
    std::string wristY = prefix + "_WRIST_Y_LINK";
    std::string wristR = boost::algorithm::to_lower_copy(prefix) + "_wrist";
    res[wristY + "_sub0"] = {wristY, prefix + "_WRIST_Y_sub0"};
    res[wristR + "_sub0"] = {wristR, prefix + "_WRIST_R_sub0"};
    res[wristR + "_sub1"] = {wristR, prefix + "_WRIST_R_sub1"};
  };
  addWristSubConvex("L");
  addWristSubConvex("R");

  return res;
}

PandaDefaultRobotModule::PandaDefaultRobotModule()
{
  readUrdf("panda", {});
  init();
}

PandaWithHandRobotModule::PandaWithHandRobotModule()
{
  readUrdf("panda", gripperLinks);
  init();
}


} // namespace mc_robots
