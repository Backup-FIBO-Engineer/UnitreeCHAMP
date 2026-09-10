// Offline checks for Unitree Go2 / B2 LowCmd motor indices and joint
// limits. No ROS.
//
// Unitree motor_cmd[0..11] = FR, FL, RR, RL (hip, thigh, calf).
// CHAMP JointState order is FL, FR, RL, RR.

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>

#include "unitree_lowcmd_map.h"
#include "motor_crc.h"

namespace
{
int g_fails = 0;
int g_checks = 0;

void check(bool ok, const char * label, const char * detail = "")
{
  ++g_checks;
  if (ok) {
    std::printf("[PASS] %s%s%s\n", label, detail[0] ? ": " : "", detail);
  } else {
    ++g_fails;
    std::printf("[FAIL] %s%s%s\n", label, detail[0] ? ": " : "", detail);
  }
}

bool stand_pose_inside(const UnitreeJointLimits & limits, const float stand[3])
{
  bool ok = true;
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      ok = ok && (unitreeClampJoint(limits, leg * 3 + j, stand[j]) == stand[j]);
    }
  }
  return ok;
}
}  // namespace

int main()
{
  check(unitreeMotorIndex("FR_hip_joint") == 0, "FR hip is motor 0");
  check(unitreeMotorIndex("FR_thigh_joint") == 1, "FR thigh is motor 1");
  check(unitreeMotorIndex("FR_calf_joint") == 2, "FR calf is motor 2");
  check(unitreeMotorIndex("FL_hip_joint") == 3, "FL hip is motor 3");
  check(unitreeMotorIndex("FL_thigh_joint") == 4, "FL thigh is motor 4");
  check(unitreeMotorIndex("FL_calf_joint") == 5, "FL calf is motor 5");
  check(unitreeMotorIndex("RR_hip_joint") == 6, "RR hip is motor 6");
  check(unitreeMotorIndex("RR_thigh_joint") == 7, "RR thigh is motor 7");
  check(unitreeMotorIndex("RR_calf_joint") == 8, "RR calf is motor 8");
  check(unitreeMotorIndex("RL_hip_joint") == 9, "RL hip is motor 9");
  check(unitreeMotorIndex("RL_thigh_joint") == 10, "RL thigh is motor 10");
  check(unitreeMotorIndex("RL_calf_joint") == 11, "RL calf is motor 11");
  check(unitreeMotorIndex("lf_hip_joint") == -1, "XGO names are not Unitree motors");
  check(unitreeMotorIndex("FL_foot_joint") == -1, "fixed foot joint is not a motor");

  std::unordered_set<std::string> unique;
  bool all_unique = true;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    all_unique = all_unique && unique.insert(kUnitreeMotorJointNames[i]).second;
  }
  check(unique.size() == static_cast<size_t>(kUnitreeMotorCount) && all_unique,
    "12 motor names are unique");

  // CHAMP publishes FL,FR,RL,RR; filling LowCmd by name must not keep that order.
  const char * champ_order[12] = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  };
  float champ_q[12];
  for (int i = 0; i < 12; ++i) {
    champ_q[i] = static_cast<float>(i);
  }
  float motor_q[12];
  for (int i = 0; i < 12; ++i) {
    motor_q[i] = -1.0f;
  }
  for (int i = 0; i < 12; ++i) {
    motor_q[unitreeMotorIndex(champ_order[i])] = champ_q[i];
  }
  check(motor_q[0] == 3.0f && motor_q[3] == 0.0f && motor_q[6] == 9.0f && motor_q[9] == 6.0f,
    "CHAMP FL/FR/RL/RR positions land on FR/FL/RR/RL motors",
    "FR0=FL? no; motor[0]=CHAMP FR (index 3)");

  check(kChampIndexFromFootForce[0] == 1, "foot_force FR -> CHAMP RF");
  check(kChampIndexFromFootForce[1] == 0, "foot_force FL -> CHAMP LF");
  check(kChampIndexFromFootForce[2] == 3, "foot_force RR -> CHAMP RH");
  check(kChampIndexFromFootForce[3] == 2, "foot_force RL -> CHAMP LH");
  bool unique_contacts = true;
  bool seen_contact[4] = {false, false, false, false};
  for (int i = 0; i < 4; ++i) {
    const int champ = kChampIndexFromFootForce[i];
    if (champ < 0 || champ > 3 || seen_contact[champ]) {
      unique_contacts = false;
    } else {
      seen_contact[champ] = true;
    }
  }
  check(unique_contacts, "foot_force map covers LF RF LH RH once");

  // Robot selection.
  UnitreeRobot robot = UnitreeRobot::kGo2;
  check(unitreeRobotFromName("b2", robot) && robot == UnitreeRobot::kB2, "robot 'b2' parses");
  check(unitreeRobotFromName("go2", robot) && robot == UnitreeRobot::kGo2, "robot 'go2' parses");
  check(!unitreeRobotFromName("", robot), "empty robot name is rejected");
  check(!unitreeRobotFromName("xgo", robot), "unknown robot name is rejected");
  check(unitreeDefaultMotorMode(UnitreeRobot::kGo2) == 0x01, "Go2 default motor mode 0x01");
  check(unitreeDefaultMotorMode(UnitreeRobot::kB2) == 0x0A, "B2 default motor mode 0x0A");
  check(std::strcmp(unitreeBaseLink(UnitreeRobot::kGo2), "base") == 0, "Go2 root link base");
  check(std::strcmp(unitreeBaseLink(UnitreeRobot::kB2), "base_link") == 0, "B2 root link base_link");
  check(&unitreeJointLimits(UnitreeRobot::kB2) == &kB2JointLimits, "B2 selects B2 limits");
  check(&unitreeJointLimits(UnitreeRobot::kGo2) == &kGo2JointLimits, "Go2 selects Go2 limits");

  // Go2 joint limits (urdf/go2.urdf): hip +-1.0472, front thigh [-1.5708, 3.4907],
  // rear thigh [-0.5236, 4.5379], calf [-2.7227, -0.83776].
  const UnitreeJointLimits & go2 = kGo2JointLimits;
  bool limits_ordered = true;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    limits_ordered = limits_ordered && (go2.lower[i] < go2.upper[i]);
  }
  check(limits_ordered, "Go2 joint limit arrays have lower < upper");
  check(unitreeClampJoint(go2, 0, 0.5f) == 0.5f, "Go2 inside range is unchanged");
  check(unitreeClampJoint(go2, 0, 2.0f) == 1.0472f, "Go2 FR hip clamps to +1.0472");
  check(unitreeClampJoint(go2, 3, -2.0f) == -1.0472f, "Go2 FL hip clamps to -1.0472");
  check(unitreeClampJoint(go2, 1, -1.7f) == -1.5708f, "Go2 FR thigh clamps to front lower -1.5708");
  check(unitreeClampJoint(go2, 7, -1.7f) == -0.5236f, "Go2 RR thigh clamps to rear lower -0.5236");
  check(unitreeClampJoint(go2, 10, 5.0f) == 4.5379f, "Go2 RL thigh clamps to rear upper 4.5379");
  check(unitreeClampJoint(go2, 2, 0.0f) == -0.83776f, "Go2 FR calf 0 rad clamps to -0.83776");
  check(unitreeClampJoint(go2, 11, -3.0f) == -2.7227f, "Go2 RL calf clamps to -2.7227");
  // CHAMP standing pose at nominal_height 0.30 must pass through untouched.
  const float go2_stand[3] = {0.0f, 0.789464891f, -1.57892966f};
  check(stand_pose_inside(go2, go2_stand), "CHAMP Go2 standing pose is inside Go2 limits");

  // B2 joint limits (urdf/b2.urdf): hip +-0.87, thigh [-0.94, 4.69],
  // calf [-2.82, -0.43], identical on all four legs.
  const UnitreeJointLimits & b2 = kB2JointLimits;
  limits_ordered = true;
  bool legs_identical = true;
  for (int i = 0; i < kUnitreeMotorCount; ++i) {
    limits_ordered = limits_ordered && (b2.lower[i] < b2.upper[i]);
    legs_identical = legs_identical &&
      b2.lower[i] == b2.lower[i % 3] && b2.upper[i] == b2.upper[i % 3];
  }
  check(limits_ordered, "B2 joint limit arrays have lower < upper");
  check(legs_identical, "B2 limits are the same on all four legs");
  check(unitreeClampJoint(b2, 0, 0.5f) == 0.5f, "B2 inside range is unchanged");
  check(unitreeClampJoint(b2, 0, 1.0f) == 0.87f, "B2 FR hip clamps to +0.87");
  check(unitreeClampJoint(b2, 3, -1.0f) == -0.87f, "B2 FL hip clamps to -0.87");
  check(unitreeClampJoint(b2, 1, -1.2f) == -0.94f, "B2 FR thigh clamps to -0.94");
  check(unitreeClampJoint(b2, 10, 5.0f) == 4.69f, "B2 RL thigh clamps to 4.69");
  check(unitreeClampJoint(b2, 2, 0.0f) == -0.43f, "B2 FR calf 0 rad clamps to -0.43");
  check(unitreeClampJoint(b2, 11, -3.0f) == -2.82f, "B2 RL calf clamps to -2.82");
  // Go2 hip limit (1.0472) is outside B2 hip range and must be clamped for B2.
  check(unitreeClampJoint(b2, 9, 1.0472f) == 0.87f, "Go2 hip limit is clamped on B2");
  // CHAMP B2 standing pose at nominal_height 0.50 (verify_champ_b2.cpp).
  const float b2_stand[3] = {0.000173972046f, 0.775150955f, -1.55030179f};
  check(stand_pose_inside(b2, b2_stand), "CHAMP B2 standing pose is inside B2 limits");
  // unitree_sdk2 b2_stand_example poses must also be inside.
  const float b2_example_lie[3] = {0.0f, 1.36f, -2.65f};
  const float b2_example_stand[3] = {0.0f, 0.67f, -1.3f};
  check(stand_pose_inside(b2, b2_example_lie), "b2_stand_example lie pose inside B2 limits");
  check(stand_pose_inside(b2, b2_example_stand), "b2_stand_example stand pose inside B2 limits");

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
