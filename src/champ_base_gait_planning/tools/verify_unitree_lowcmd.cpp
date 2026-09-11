// Offline checks of the rt/lowcmd mapping for any Unitree robot described by
// its URDF and CHAMP joints yaml. No ROS: urdfdom (urdf_parser) + yaml-cpp.
//
//   verify_unitree_lowcmd <urdf> <joints.yaml>
//
// Unitree motor_cmd[0..11] = FR, FL, RR, RL (hip, thigh, calf).
// CHAMP JointState order is LF, RF, LH, RH.

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

#include "unitree_lowcmd_map.h"
#include "motor_crc.h"

namespace
{
int g_fails = 0;
int g_checks = 0;

std::string fmt(const char * format, ...)
{
  char buf[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  return std::string(buf);
}

void check(bool ok, const std::string & label, const std::string & detail = "")
{
  ++g_checks;
  std::printf(
    "[%s] %s%s%s\n", ok ? "PASS" : "FAIL", label.c_str(), detail.empty() ? "" : ": ",
    detail.c_str());
  if (!ok) {
    ++g_fails;
  }
}

// CHAMP joints_map legs in yaml, keyed by leg name.
std::vector<std::string> legJoints(const YAML::Node & params, const char * leg)
{
  const YAML::Node list = params["joints_map"][leg];
  if (!list || !list.IsSequence()) {
    throw std::runtime_error(std::string("joints_map.") + leg + " missing");
  }
  std::vector<std::string> names;
  for (const auto & n : list) {
    names.push_back(n.as<std::string>());
  }
  return names;
}

YAML::Node loadRosParams(const std::string & path)
{
  YAML::Node doc = YAML::LoadFile(path);
  for (auto it = doc.begin(); it != doc.end(); ++it) {
    if (it->second["ros__parameters"]) {
      return it->second["ros__parameters"];
    }
  }
  throw std::runtime_error(path + ": no ros__parameters block");
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <urdf> <joints.yaml>\n", argv[0]);
    return 2;
  }
  std::ifstream urdf_file(argv[1]);
  std::stringstream urdf_xml;
  urdf_xml << urdf_file.rdbuf();
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml.str());
  if (!model) {
    std::fprintf(stderr, "cannot parse URDF %s\n", argv[1]);
    return 2;
  }
  const YAML::Node params = loadRosParams(argv[2]);

  std::array<std::vector<std::string>, kUnitreeLegCount> legs;
  for (int i = 0; i < kUnitreeLegCount; ++i) {
    legs[static_cast<size_t>(i)] = legJoints(params, kUnitreeLegKeys[i]);
  }
  const UnitreeMotorMap map = unitreeMotorMapFromLegs(legs);
  const UnitreeJointLimits limits = unitreeJointLimitsFromUrdf(*model, map);
  std::printf("Robot '%s': rt/lowcmd motor -> joint\n", model->getName().c_str());
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    std::printf(
      "  motor %2d = %-20s [%8.4f, %8.4f]\n", i, map.joint_names[static_cast<size_t>(i)].c_str(),
      static_cast<double>(limits.lower[static_cast<size_t>(i)]),
      static_cast<double>(limits.upper[static_cast<size_t>(i)]));
  }

  // Protocol layout: leg order FR, FL, RR, RL; joint order hip, thigh, calf.
  check(std::string(kUnitreeLegKeys[0]) == "right_front", "motor 0..2 is the right front leg");
  check(std::string(kUnitreeLegKeys[1]) == "left_front", "motor 3..5 is the left front leg");
  check(std::string(kUnitreeLegKeys[2]) == "right_hind", "motor 6..8 is the right hind leg");
  check(std::string(kUnitreeLegKeys[3]) == "left_hind", "motor 9..11 is the left hind leg");
  for (int leg = 0; leg < kUnitreeLegCount; ++leg) {
    for (int j = 0; j < kUnitreeJointsPerLeg; ++j) {
      const std::string & name = legs[static_cast<size_t>(leg)][static_cast<size_t>(j)];
      check(
        map.motorIndex(name) == leg * kUnitreeJointsPerLeg + j,
        fmt("%s joint %d (%s) is motor %d", kUnitreeLegKeys[leg], j, name.c_str(),
          leg * kUnitreeJointsPerLeg + j));
    }
    // The fourth joints_map entry is the fixed foot joint and must not be a motor.
    if (legs[static_cast<size_t>(leg)].size() > 3) {
      check(
        map.motorIndex(legs[static_cast<size_t>(leg)][3]) == -1,
        fmt("%s foot joint '%s' is not a motor", kUnitreeLegKeys[leg],
          legs[static_cast<size_t>(leg)][3].c_str()));
    }
  }
  check(map.motorIndex("not_a_joint") == -1, "unknown joint name is rejected");

  std::unordered_set<std::string> unique(map.joint_names.begin(), map.joint_names.end());
  check(unique.size() == static_cast<size_t>(kUnitreeMotorCount), "12 motor names are unique");

  // Every motor joint is a revolute URDF joint with a numeric range.
  bool limits_ordered = true;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    limits_ordered = limits_ordered &&
      limits.lower[static_cast<size_t>(i)] < limits.upper[static_cast<size_t>(i)];
  }
  check(limits_ordered, "URDF joint limits have lower < upper for all 12 motors");

  // Clamping: inside untouched, outside pinned to the bound, both sides.
  bool clamp_ok = true;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    const float lo = limits.lower[static_cast<size_t>(i)];
    const float hi = limits.upper[static_cast<size_t>(i)];
    const float mid = 0.5f * (lo + hi);
    clamp_ok = clamp_ok && unitreeClampJoint(limits, i, mid) == mid;
    clamp_ok = clamp_ok && unitreeClampJoint(limits, i, lo) == lo;
    clamp_ok = clamp_ok && unitreeClampJoint(limits, i, hi) == hi;
    clamp_ok = clamp_ok && unitreeClampJoint(limits, i, hi + 1.0f) == hi;
    clamp_ok = clamp_ok && unitreeClampJoint(limits, i, lo - 1.0f) == lo;
  }
  check(clamp_ok, "clamp keeps in-range targets and pins out-of-range targets to the URDF limit");

  // CHAMP publishes LF,FR,LH,RH; filling LowCmd by name must reorder to FR,FL,RR,RL.
  const char * champ_order[kUnitreeLegCount] = {"left_front", "right_front", "left_hind", "right_hind"};
  float motor_q[kUnitreeMotorCount];
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    motor_q[i] = -1.0f;
  }
  for (int leg = 0; leg < kUnitreeLegCount; ++leg) {
    const YAML::Node list = params["joints_map"][champ_order[leg]];
    for (int j = 0; j < kUnitreeJointsPerLeg; ++j) {
      const int idx = map.motorIndex(list[j].as<std::string>());
      motor_q[idx] = static_cast<float>(leg * kUnitreeJointsPerLeg + j);
    }
  }
  check(
    motor_q[0] == 3.0f && motor_q[3] == 0.0f && motor_q[6] == 9.0f && motor_q[9] == 6.0f,
    "CHAMP LF/RF/LH/RH positions land on FL/FR/RL/RR motor slots");

  check(kChampLegFromUnitreeLeg[0] == 1, "foot_force FR -> CHAMP RF");
  check(kChampLegFromUnitreeLeg[1] == 0, "foot_force FL -> CHAMP LF");
  check(kChampLegFromUnitreeLeg[2] == 3, "foot_force RR -> CHAMP RH");
  check(kChampLegFromUnitreeLeg[3] == 2, "foot_force RL -> CHAMP LH");
  bool seen_contact[kUnitreeLegCount] = {false, false, false, false};
  bool unique_contacts = true;
  for (int i = 0; i < kUnitreeLegCount; ++i) {
    const int champ = kChampLegFromUnitreeLeg[i];
    if (champ < 0 || champ >= kUnitreeLegCount || seen_contact[champ]) {
      unique_contacts = false;
    } else {
      seen_contact[champ] = true;
    }
  }
  check(unique_contacts, "foot_force map covers LF RF LH RH once");

  // A joints_map with a missing or duplicated name must be rejected up front.
  {
    auto broken = legs;
    broken[0].resize(2);
    bool threw = false;
    try {
      unitreeMotorMapFromLegs(broken);
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "joints_map leg with fewer than 3 joints is rejected");
    broken = legs;
    broken[0][0] = broken[1][0];
    threw = false;
    try {
      unitreeMotorMapFromLegs(broken);
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "duplicated joint name across legs is rejected");
    UnitreeMotorMap bogus = map;
    bogus.joint_names[0] = "not_in_urdf";
    threw = false;
    try {
      unitreeJointLimitsFromUrdf(*model, bogus);
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "joint missing from the URDF is rejected");
  }

  uint32_t word = 0;
  const uint32_t crc_zero = crc32_core(&word, 1);
  check(crc_zero != 0u, "CRC of a zero word is not 0");
  check(crc32_core(&word, 1) == crc_zero, "CRC is deterministic");
  word = 1;
  check(crc32_core(&word, 1) != crc_zero, "CRC changes when the payload changes");
  check(LOWLEVEL == 0xff, "LowCmd level_flag is 0xFF");

  std::printf("\nResult: %d failed / %d checks\n", g_fails, g_checks);
  return g_fails ? 1 : 0;
}
