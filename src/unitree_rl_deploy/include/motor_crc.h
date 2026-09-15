#ifndef UNITREE_RL_MOTOR_CRC_H
#define UNITREE_RL_MOTOR_CRC_H

#include <cstdint>

// Same CRC32 Unitree uses on LowCmd (poly 0x04c11db7, no table, MSB first).
uint32_t crc32_core(uint32_t * ptr, uint32_t len);

constexpr int LOWLEVEL = 0xff;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// Byte layout the robot computes the LowCmd CRC over. It is the natural C
// layout of the unitree_go LowCmd IDL struct (unitree_ros2
// example/src/include/common/motor_crc.h). The ROS 2 message
// unitree_go/msg/LowCmd is repacked into this struct before the CRC because
// the generated ROS message type does not have the wire layout. This header
// stays ROS-free so the offline checks can compile it without ROS.
struct UnitreeLowCmdWire
{
  struct MotorCmd
  {
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float kp;
    float kd;
    uint32_t reserve[3];
  };
  struct BmsCmd
  {
    uint8_t off;
    uint8_t reserve[3];
  };

  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t bandwidth;
  MotorCmd motor_cmd[20];
  BmsCmd bms_cmd;
  uint8_t wireless_remote[40];
  uint8_t led[12];
  uint8_t fan[2];
  uint8_t gpio;
  uint32_t reserve;
  uint32_t crc;
};

constexpr int kUnitreeLowCmdMotorSlots = 20;
static_assert(sizeof(UnitreeLowCmdWire::MotorCmd) == 36, "MotorCmd wire size");
static_assert(sizeof(UnitreeLowCmdWire) == 812, "LowCmd wire size");
// Every 32-bit word before the trailing crc field.
constexpr uint32_t kUnitreeLowCmdCrcWords = (sizeof(UnitreeLowCmdWire) >> 2) - 1;

inline uint32_t unitreeLowCmdWireCrc(UnitreeLowCmdWire & wire)
{
  return crc32_core(reinterpret_cast<uint32_t *>(&wire), kUnitreeLowCmdCrcWords);
}

#endif
