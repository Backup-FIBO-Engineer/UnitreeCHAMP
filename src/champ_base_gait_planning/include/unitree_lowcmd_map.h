#ifndef UNITREE_LOWCMD_MAP_H
#define UNITREE_LOWCMD_MAP_H

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <urdf_model/model.h>

// Unitree quadrupeds publish/consume the unitree_go LowCmd_/LowState_ IDL on
// rt/lowcmd / rt/lowstate. motor_cmd[0..11], motor_state[0..11] and
// foot_force[0..3] are laid out by leg FR, FL, RR, RL and, inside a leg, hip,
// thigh, calf (unitree_sdk2 go2/b2 examples). That layout is protocol, not
// robot geometry: the joint names behind each slot come from the CHAMP
// joints_map yaml and the joint limits from the URDF of the selected robot.
inline constexpr int kUnitreeLegCount = 4;
inline constexpr int kUnitreeJointsPerLeg = 3;
inline constexpr int kUnitreeMotorCount = kUnitreeLegCount * kUnitreeJointsPerLeg;

// joints_map / links_map keys in protocol leg order.
inline constexpr const char * kUnitreeLegKeys[kUnitreeLegCount] = {
  "right_front", "left_front", "right_hind", "left_hind",
};

// CHAMP leg arrays are LF, RF, LH, RH. kChampLegFromUnitreeLeg[i] is the CHAMP
// index of protocol leg i (FR->1, FL->0, RR->3, RL->2). Used for foot_force.
inline constexpr int kChampLegFromUnitreeLeg[kUnitreeLegCount] = {1, 0, 3, 2};

struct UnitreeMotorMap
{
  std::array<std::string, kUnitreeMotorCount> joint_names;

  int motorIndex(const std::string & joint_name) const
  {
    for (int i = 0; i < kUnitreeMotorCount; ++i) {
      if (joint_names[static_cast<size_t>(i)] == joint_name) {
        return i;
      }
    }
    return -1;
  }
};

struct UnitreeJointLimits
{
  std::array<float, kUnitreeMotorCount> lower{};
  std::array<float, kUnitreeMotorCount> upper{};
};

// legs[i] is the joints_map entry for kUnitreeLegKeys[i]; the first three names
// are the actuated hip, thigh, calf joints (the fourth is the fixed foot joint).
inline UnitreeMotorMap unitreeMotorMapFromLegs(
  const std::array<std::vector<std::string>, kUnitreeLegCount> & legs)
{
  UnitreeMotorMap map;
  for (int leg = 0; leg < kUnitreeLegCount; ++leg) {
    const auto & names = legs[static_cast<size_t>(leg)];
    if (names.size() < static_cast<size_t>(kUnitreeJointsPerLeg)) {
      throw std::runtime_error(
              std::string("joints_map.") + kUnitreeLegKeys[leg] +
              " needs hip, thigh and calf joint names");
    }
    for (int j = 0; j < kUnitreeJointsPerLeg; ++j) {
      map.joint_names[static_cast<size_t>(leg * kUnitreeJointsPerLeg + j)] =
        names[static_cast<size_t>(j)];
    }
  }
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    for (int j = i + 1; j < kUnitreeMotorCount; ++j) {
      if (map.joint_names[static_cast<size_t>(i)] == map.joint_names[static_cast<size_t>(j)]) {
        throw std::runtime_error(
                "joints_map lists joint '" + map.joint_names[static_cast<size_t>(i)] +
                "' twice");
      }
    }
  }
  return map;
}

// Position limits of every motor joint, read from the URDF <limit lower upper>.
inline UnitreeJointLimits unitreeJointLimitsFromUrdf(
  const urdf::ModelInterface & model, const UnitreeMotorMap & map)
{
  UnitreeJointLimits limits;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    const std::string & name = map.joint_names[static_cast<size_t>(i)];
    const urdf::JointConstSharedPtr joint = model.getJoint(name);
    if (!joint) {
      throw std::runtime_error("URDF has no joint '" + name + "' listed in joints_map");
    }
    if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS) {
      throw std::runtime_error("URDF joint '" + name + "' is not revolute");
    }
    if (!joint->limits || !(joint->limits->lower < joint->limits->upper)) {
      throw std::runtime_error(
              "URDF joint '" + name + "' needs <limit lower upper> for rt/lowcmd clamping");
    }
    limits.lower[static_cast<size_t>(i)] = static_cast<float>(joint->limits->lower);
    limits.upper[static_cast<size_t>(i)] = static_cast<float>(joint->limits->upper);
  }
  return limits;
}

inline float unitreeClampJoint(
  const UnitreeJointLimits & limits, int motor_index, float q)
{
  const size_t i = static_cast<size_t>(motor_index);
  if (q < limits.lower[i]) {
    return limits.lower[i];
  }
  if (q > limits.upper[i]) {
    return limits.upper[i];
  }
  return q;
}

#endif
