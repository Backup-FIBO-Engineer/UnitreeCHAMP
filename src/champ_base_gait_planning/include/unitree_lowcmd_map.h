#ifndef UNITREE_LOWCMD_MAP_H
#define UNITREE_LOWCMD_MAP_H

#include <cstdint>
#include <string>

// Unitree Go2 and B2 share the unitree_go LowCmd_/LowState_ IDL on
// rt/lowcmd / rt/lowstate. motor_cmd[0..11] and foot_force[0..3] order is
// FR, FL, RR, RL. CHAMP JointState / ContactsStamped is FL, FR, RL, RR.
inline constexpr int kUnitreeMotorCount = 12;

inline constexpr const char * kUnitreeMotorJointNames[kUnitreeMotorCount] = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

// foot_force[i] (FR,FL,RR,RL) -> ContactsStamped index (LF,RF,LH,RH)
inline constexpr int kChampIndexFromFootForce[4] = {1, 0, 3, 2};

enum class UnitreeRobot
{
  kGo2,
  kB2,
};

struct UnitreeJointLimits
{
  float lower[kUnitreeMotorCount];
  float upper[kUnitreeMotorCount];
};

// Joint limits from urdf/go2.urdf, in motor order. Front and rear thighs
// differ. The MuJoCo sim clamps ctrl to these; the real robot does not.
inline constexpr UnitreeJointLimits kGo2JointLimits = {
  {
    -1.0472f, -1.5708f, -2.7227f,
    -1.0472f, -1.5708f, -2.7227f,
    -1.0472f, -0.5236f, -2.7227f,
    -1.0472f, -0.5236f, -2.7227f,
  },
  {
    1.0472f, 3.4907f, -0.83776f,
    1.0472f, 3.4907f, -0.83776f,
    1.0472f, 4.5379f, -0.83776f,
    1.0472f, 4.5379f, -0.83776f,
  },
};

// Joint limits from urdf/b2.urdf (unitree_ros b2_description), in motor
// order. All four legs share the same limits.
inline constexpr UnitreeJointLimits kB2JointLimits = {
  {
    -0.87f, -0.94f, -2.82f,
    -0.87f, -0.94f, -2.82f,
    -0.87f, -0.94f, -2.82f,
    -0.87f, -0.94f, -2.82f,
  },
  {
    0.87f, 4.69f, -0.43f,
    0.87f, 4.69f, -0.43f,
    0.87f, 4.69f, -0.43f,
    0.87f, 4.69f, -0.43f,
  },
};

inline const UnitreeJointLimits & unitreeJointLimits(UnitreeRobot robot)
{
  return robot == UnitreeRobot::kB2 ? kB2JointLimits : kGo2JointLimits;
}

// motor_cmd.mode used by the official unitree_sdk2 stand examples:
// go2_stand_example 0x01, b2_stand_example 0x0A.
inline uint8_t unitreeDefaultMotorMode(UnitreeRobot robot)
{
  return robot == UnitreeRobot::kB2 ? 0x0A : 0x01;
}

inline const char * unitreeRobotName(UnitreeRobot robot)
{
  return robot == UnitreeRobot::kB2 ? "B2" : "Go2";
}

// URDF root link: urdf/go2.urdf "base", urdf/b2.urdf "base_link".
inline const char * unitreeBaseLink(UnitreeRobot robot)
{
  return robot == UnitreeRobot::kB2 ? "base_link" : "base";
}

inline bool unitreeRobotFromName(const std::string & name, UnitreeRobot & robot)
{
  if (name == "go2" || name == "Go2" || name == "GO2") {
    robot = UnitreeRobot::kGo2;
    return true;
  }
  if (name == "b2" || name == "B2") {
    robot = UnitreeRobot::kB2;
    return true;
  }
  return false;
}

inline float unitreeClampJoint(
  const UnitreeJointLimits & limits, int motor_index, float q)
{
  if (q < limits.lower[motor_index]) {
    return limits.lower[motor_index];
  }
  if (q > limits.upper[motor_index]) {
    return limits.upper[motor_index];
  }
  return q;
}

inline int unitreeMotorIndex(const std::string & joint_name)
{
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    if (joint_name == kUnitreeMotorJointNames[i]) {
      return i;
    }
  }
  return -1;
}

#endif
