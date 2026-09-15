#ifndef UNITREE_RL_LOWCMD_CRC_H
#define UNITREE_RL_LOWCMD_CRC_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>

#include <unitree_go/msg/bms_cmd.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/motor_cmd.hpp>

#include "motor_crc.h"

// ROS 2 array extents must match the wire struct: std::copy below is not
// bounds-checked, and a longer ROS field would overflow the CRC buffer.
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::head)>::value == 2);
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::sn)>::value == 2);
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::version)>::value == 2);
static_assert(
  std::tuple_size<decltype(unitree_go::msg::LowCmd::motor_cmd)>::value ==
  static_cast<std::size_t>(kUnitreeLowCmdMotorSlots));
static_assert(std::tuple_size<decltype(unitree_go::msg::MotorCmd::reserve)>::value == 3);
static_assert(std::tuple_size<decltype(unitree_go::msg::BmsCmd::reserve)>::value == 3);
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::wireless_remote)>::value == 40);
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::led)>::value == 12);
static_assert(std::tuple_size<decltype(unitree_go::msg::LowCmd::fan)>::value == 2);

// CRC of a unitree_go/msg/LowCmd, computed the way the robot checks it
// (unitree_ros2 example/src/src/common/motor_crc.cpp get_crc()): the ROS 2
// message is repacked into the C wire struct and crc32_core runs over every
// word but the last.
inline uint32_t unitreeLowCmdCrc(const unitree_go::msg::LowCmd & msg)
{
  UnitreeLowCmdWire raw{};
  std::copy(msg.head.begin(), msg.head.end(), raw.head);
  raw.level_flag = msg.level_flag;
  raw.frame_reserve = msg.frame_reserve;
  std::copy(msg.sn.begin(), msg.sn.end(), raw.sn);
  std::copy(msg.version.begin(), msg.version.end(), raw.version);
  raw.bandwidth = msg.bandwidth;
  for (int i = 0; i < kUnitreeLowCmdMotorSlots; ++i) {
    const auto & cmd = msg.motor_cmd[static_cast<size_t>(i)];
    auto & wire = raw.motor_cmd[i];
    wire.mode = cmd.mode;
    wire.q = cmd.q;
    wire.dq = cmd.dq;
    wire.tau = cmd.tau;
    wire.kp = cmd.kp;
    wire.kd = cmd.kd;
    std::copy(cmd.reserve.begin(), cmd.reserve.end(), wire.reserve);
  }
  raw.bms_cmd.off = msg.bms_cmd.off;
  std::copy(msg.bms_cmd.reserve.begin(), msg.bms_cmd.reserve.end(), raw.bms_cmd.reserve);
  std::copy(msg.wireless_remote.begin(), msg.wireless_remote.end(), raw.wireless_remote);
  std::copy(msg.led.begin(), msg.led.end(), raw.led);
  std::copy(msg.fan.begin(), msg.fan.end(), raw.fan);
  raw.gpio = msg.gpio;
  raw.reserve = msg.reserve;
  return unitreeLowCmdWireCrc(raw);
}

inline void unitreeSetLowCmdCrc(unitree_go::msg::LowCmd & msg)
{
  msg.crc = unitreeLowCmdCrc(msg);
}

#endif
