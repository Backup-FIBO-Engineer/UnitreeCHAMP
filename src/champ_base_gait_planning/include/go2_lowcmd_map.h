#ifndef GO2_LOWCMD_MAP_H
#define GO2_LOWCMD_MAP_H

#include <string>

// Unitree Go2 DDS rt/lowcmd motor_cmd[0..11] and foot_force[0..3] order:
// FR, FL, RR, RL. CHAMP JointState / ContactsStamped is FL, FR, RL, RR.
inline constexpr int kGo2MotorCount = 12;

inline constexpr const char * kGo2MotorJointNames[kGo2MotorCount] = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

// foot_force[i] (FR,FL,RR,RL) -> ContactsStamped index (LF,RF,LH,RH)
inline constexpr int kChampIndexFromFootForce[4] = {1, 0, 3, 2};

inline int go2MotorIndex(const std::string & joint_name)
{
  for (int i = 0; i < kGo2MotorCount; ++i) {
    if (joint_name == kGo2MotorJointNames[i]) {
      return i;
    }
  }
  return -1;
}

#endif
