// Robot description for the offline CHAMP tools: URDF (tinyxml2) + the CHAMP
// gait/joints/links yaml files (yaml-cpp). No ROS. Replicates what
// champ::URDF::loadFromString does at runtime so the checks exercise the same
// numbers the nodes will use, for any robot described by those files.
#ifndef CHAMP_ROBOT_CONFIG_H
#define CHAMP_ROBOT_CONFIG_H

#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <quadruped_base/quadruped_base.h>

namespace champ_tools
{

constexpr int kLegCount = 4;
constexpr int kChainLength = 4;  // hip, upper leg, lower leg, foot
constexpr int kJointsPerLeg = 3;
// CHAMP leg order (QuadrupedBase::legs): LF, RF, LH, RH.
const char * const kLegKeys[kLegCount] = {"left_front", "right_front", "left_hind", "right_hind"};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct UrdfJoint
{
  std::string name;
  std::string type;
  std::string parent;
  std::string child;
  Vec3 xyz;
  Vec3 rpy;
  Vec3 axis{1.0, 0.0, 0.0};
  bool has_limits{false};
  double lower{0.0};
  double upper{0.0};
  double effort{0.0};
  double velocity{0.0};
};

struct Urdf
{
  std::string name;
  std::string root;
  std::map<std::string, UrdfJoint> joints;
  std::map<std::string, std::string> parent_joint_of_link;  // child link -> joint name
  std::vector<std::string> links;

  const UrdfJoint & joint(const std::string & joint_name) const
  {
    const auto it = joints.find(joint_name);
    if (it == joints.end()) {
      throw std::runtime_error("URDF has no joint '" + joint_name + "'");
    }
    return it->second;
  }

  bool hasLink(const std::string & link) const
  {
    for (const auto & l : links) {
      if (l == link) {
        return true;
      }
    }
    return false;
  }

  // champ::URDF::getPose: sum parent_to_joint_origin xyz from end_link up to ref_link.
  Vec3 chainXyz(const std::string & ref_link, const std::string & end_link) const
  {
    Vec3 total;
    std::string current = end_link;
    while (current != ref_link) {
      const auto it = parent_joint_of_link.find(current);
      if (it == parent_joint_of_link.end()) {
        throw std::runtime_error(
                "link '" + end_link + "' is not below '" + ref_link + "' in the URDF");
      }
      const UrdfJoint & j = joints.at(it->second);
      total.x += j.xyz.x;
      total.y += j.xyz.y;
      total.z += j.xyz.z;
      current = j.parent;
    }
    return total;
  }
};

inline Vec3 parseVec3(const char * text, Vec3 fallback = Vec3{})
{
  if (text == nullptr) {
    return fallback;
  }
  Vec3 v;
  if (std::sscanf(text, "%lf %lf %lf", &v.x, &v.y, &v.z) != 3) {
    throw std::runtime_error(std::string("bad vector '") + text + "'");
  }
  return v;
}

inline Urdf loadUrdf(const std::string & path)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot parse URDF " + path + ": " + doc.ErrorStr());
  }
  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::runtime_error(path + " has no <robot> element");
  }
  Urdf urdf;
  urdf.name = robot->Attribute("name") ? robot->Attribute("name") : "";
  for (const tinyxml2::XMLElement * link = robot->FirstChildElement("link"); link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    urdf.links.emplace_back(link->Attribute("name") ? link->Attribute("name") : "");
  }
  for (const tinyxml2::XMLElement * joint = robot->FirstChildElement("joint"); joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    UrdfJoint j;
    j.name = joint->Attribute("name") ? joint->Attribute("name") : "";
    j.type = joint->Attribute("type") ? joint->Attribute("type") : "";
    const auto * parent = joint->FirstChildElement("parent");
    const auto * child = joint->FirstChildElement("child");
    if (parent == nullptr || child == nullptr) {
      throw std::runtime_error("joint '" + j.name + "' lacks parent/child");
    }
    j.parent = parent->Attribute("link");
    j.child = child->Attribute("link");
    if (const auto * origin = joint->FirstChildElement("origin")) {
      j.xyz = parseVec3(origin->Attribute("xyz"));
      j.rpy = parseVec3(origin->Attribute("rpy"));
    }
    if (const auto * axis = joint->FirstChildElement("axis")) {
      j.axis = parseVec3(axis->Attribute("xyz"), Vec3{1.0, 0.0, 0.0});
    }
    if (const auto * limit = joint->FirstChildElement("limit")) {
      j.has_limits = limit->Attribute("lower") != nullptr && limit->Attribute("upper") != nullptr;
      if (j.has_limits) {
        j.lower = limit->DoubleAttribute("lower");
        j.upper = limit->DoubleAttribute("upper");
      }
      j.effort = limit->DoubleAttribute("effort", 0.0);
      j.velocity = limit->DoubleAttribute("velocity", 0.0);
    }
    urdf.parent_joint_of_link[j.child] = j.name;
    urdf.joints[j.name] = j;
  }
  for (const auto & link : urdf.links) {
    if (urdf.parent_joint_of_link.count(link) == 0) {
      if (!urdf.root.empty()) {
        throw std::runtime_error("URDF has more than one root link: " + urdf.root + ", " + link);
      }
      urdf.root = link;
    }
  }
  if (urdf.root.empty()) {
    throw std::runtime_error("URDF has no root link");
  }
  return urdf;
}

// ROS 2 parameter yaml: merge every <node>/ros__parameters block.
inline YAML::Node loadRosParams(const std::vector<std::string> & paths)
{
  YAML::Node merged(YAML::NodeType::Map);
  for (const auto & path : paths) {
    YAML::Node doc = YAML::LoadFile(path);
    bool found = false;
    for (auto it = doc.begin(); it != doc.end(); ++it) {
      const YAML::Node params = it->second["ros__parameters"];
      if (!params || !params.IsMap()) {
        continue;
      }
      found = true;
      for (auto p = params.begin(); p != params.end(); ++p) {
        const std::string key = p->first.as<std::string>();
        if (p->second.IsMap() && merged[key] && merged[key].IsMap()) {
          for (auto q = p->second.begin(); q != p->second.end(); ++q) {
            merged[key][q->first.as<std::string>()] = q->second;
          }
        } else {
          merged[key] = p->second;
        }
      }
    }
    if (!found) {
      throw std::runtime_error(path + ": no ros__parameters block");
    }
  }
  return merged;
}

using LegChain = std::array<std::string, kChainLength>;

inline std::array<LegChain, kLegCount> loadChains(const YAML::Node & params, const char * key)
{
  const YAML::Node map = params[key];
  if (!map || !map.IsMap()) {
    throw std::runtime_error(std::string(key) + " missing from yaml");
  }
  std::array<LegChain, kLegCount> chains;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const YAML::Node list = map[kLegKeys[leg]];
    if (!list || !list.IsSequence() || list.size() < static_cast<size_t>(kChainLength)) {
      throw std::runtime_error(
              std::string(key) + "." + kLegKeys[leg] + " needs " +
              std::to_string(kChainLength) + " names");
    }
    for (int i = 0; i < kChainLength; ++i) {
      chains[static_cast<size_t>(leg)][static_cast<size_t>(i)] = list[i].as<std::string>();
    }
  }
  return chains;
}

struct GaitParams
{
  std::string knee_orientation{">>"};
  double odom_scaler{1.0};
  double max_linear_velocity_x{0.0};
  double max_linear_velocity_y{0.0};
  double max_angular_velocity_z{0.0};
  double com_x_translation{0.0};
  double swing_height{0.0};
  double stance_depth{0.0};
  double stance_duration{0.25};
  double nominal_height{0.0};
};

inline GaitParams loadGait(const YAML::Node & params)
{
  const YAML::Node gait = params["gait"];
  if (!gait || !gait.IsMap()) {
    throw std::runtime_error("gait missing from yaml");
  }
  GaitParams g;
  auto get = [&gait](const char * key, double & out) {
      if (!gait[key]) {
        throw std::runtime_error(std::string("gait.") + key + " missing from yaml");
      }
      out = gait[key].as<double>();
    };
  if (gait["knee_orientation"]) {
    g.knee_orientation = gait["knee_orientation"].as<std::string>();
  }
  if (gait["odom_scaler"]) {
    g.odom_scaler = gait["odom_scaler"].as<double>();
  }
  get("max_linear_velocity_x", g.max_linear_velocity_x);
  get("max_linear_velocity_y", g.max_linear_velocity_y);
  get("max_angular_velocity_z", g.max_angular_velocity_z);
  get("com_x_translation", g.com_x_translation);
  get("swing_height", g.swing_height);
  get("stance_depth", g.stance_depth);
  get("stance_duration", g.stance_duration);
  get("nominal_height", g.nominal_height);
  return g;
}

struct RobotConfig
{
  Urdf urdf;
  GaitParams gait;
  std::array<LegChain, kLegCount> links;
  std::array<LegChain, kLegCount> joints;
  std::string base_link;
  std::string imu_link;
  // champ::URDF::fillLeg translations: [leg][hip, upper, lower, foot].
  std::array<std::array<Vec3, kChainLength>, kLegCount> xyz;

  // The 12 actuated joint names in CHAMP order (LF, RF, LH, RH x hip, upper, lower).
  std::vector<std::string> champJointNames() const
  {
    std::vector<std::string> names;
    for (int leg = 0; leg < kLegCount; ++leg) {
      for (int j = 0; j < kJointsPerLeg; ++j) {
        names.push_back(joints[static_cast<size_t>(leg)][static_cast<size_t>(j)]);
      }
    }
    return names;
  }

  // champ::GaitConfig keeps a const char* to knee_orientation; keep this object alive.
  champ::GaitConfig champGait() const
  {
    champ::GaitConfig g;
    g.knee_orientation = gait.knee_orientation.c_str();
    g.odom_scaler = static_cast<float>(gait.odom_scaler);
    g.max_linear_velocity_x = static_cast<float>(gait.max_linear_velocity_x);
    g.max_linear_velocity_y = static_cast<float>(gait.max_linear_velocity_y);
    g.max_angular_velocity_z = static_cast<float>(gait.max_angular_velocity_z);
    g.com_x_translation = static_cast<float>(gait.com_x_translation);
    g.swing_height = static_cast<float>(gait.swing_height);
    g.stance_depth = static_cast<float>(gait.stance_depth);
    g.stance_duration = static_cast<float>(gait.stance_duration);
    g.nominal_height = static_cast<float>(gait.nominal_height);
    return g;
  }

  // Same as champ::URDF::loadFromString on a QuadrupedBase.
  void applyTo(champ::QuadrupedBase & base) const
  {
    for (int leg = 0; leg < kLegCount; ++leg) {
      for (int i = 0; i < kChainLength; ++i) {
        const Vec3 & v = xyz[static_cast<size_t>(leg)][static_cast<size_t>(i)];
        base.legs[leg]->joint_chain[i]->setTranslation(
          static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
      }
    }
  }
};

inline RobotConfig loadRobotConfig(
  const std::string & urdf_path, const std::string & gait_yaml,
  const std::string & joints_yaml, const std::string & links_yaml)
{
  RobotConfig cfg;
  cfg.urdf = loadUrdf(urdf_path);
  const YAML::Node params = loadRosParams({gait_yaml, joints_yaml, links_yaml});
  cfg.gait = loadGait(params);
  cfg.links = loadChains(params, "links_map");
  cfg.joints = loadChains(params, "joints_map");
  if (params["links_map"]["base"]) {
    cfg.base_link = params["links_map"]["base"].as<std::string>();
  }
  if (params["links_map"]["imu"]) {
    cfg.imu_link = params["links_map"]["imu"].as<std::string>();
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    const LegChain & chain = cfg.links[static_cast<size_t>(leg)];
    for (int i = kChainLength - 1; i >= 0; --i) {
      const std::string ref = i > 0 ? chain[static_cast<size_t>(i - 1)] : cfg.urdf.root;
      cfg.xyz[static_cast<size_t>(leg)][static_cast<size_t>(i)] =
        cfg.urdf.chainXyz(ref, chain[static_cast<size_t>(i)]);
    }
  }
  return cfg;
}

}  // namespace champ_tools

#endif  // CHAMP_ROBOT_CONFIG_H
