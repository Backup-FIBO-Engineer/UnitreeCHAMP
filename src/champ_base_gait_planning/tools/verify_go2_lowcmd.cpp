// Offline checks for Go2 LowCmd motor indices. No ROS.
//
// Unitree motor_cmd[0..11] = FR, FL, RR, RL (hip, thigh, calf).
// CHAMP JointState order is FL, FR, RL, RR.

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>

#include "go2_lowcmd_map.h"
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
}  // namespace

int main()
{
  check(go2MotorIndex("FR_hip_joint") == 0, "FR hip is motor 0");
  check(go2MotorIndex("FR_thigh_joint") == 1, "FR thigh is motor 1");
  check(go2MotorIndex("FR_calf_joint") == 2, "FR calf is motor 2");
  check(go2MotorIndex("FL_hip_joint") == 3, "FL hip is motor 3");
  check(go2MotorIndex("FL_thigh_joint") == 4, "FL thigh is motor 4");
  check(go2MotorIndex("FL_calf_joint") == 5, "FL calf is motor 5");
  check(go2MotorIndex("RR_hip_joint") == 6, "RR hip is motor 6");
  check(go2MotorIndex("RR_thigh_joint") == 7, "RR thigh is motor 7");
  check(go2MotorIndex("RR_calf_joint") == 8, "RR calf is motor 8");
  check(go2MotorIndex("RL_hip_joint") == 9, "RL hip is motor 9");
  check(go2MotorIndex("RL_thigh_joint") == 10, "RL thigh is motor 10");
  check(go2MotorIndex("RL_calf_joint") == 11, "RL calf is motor 11");
  check(go2MotorIndex("lf_hip_joint") == -1, "XGO names are not Go2 motors");
  check(go2MotorIndex("FL_foot_joint") == -1, "fixed foot joint is not a motor");

  std::unordered_set<std::string> unique;
  bool all_unique = true;
  for (int i = 0; i < kGo2MotorCount; ++i) {
    all_unique = all_unique && unique.insert(kGo2MotorJointNames[i]).second;
  }
  check(unique.size() == static_cast<size_t>(kGo2MotorCount) && all_unique,
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
    motor_q[go2MotorIndex(champ_order[i])] = champ_q[i];
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
