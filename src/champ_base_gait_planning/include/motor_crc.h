#ifndef CHAMP_GO2_MOTOR_CRC_H
#define CHAMP_GO2_MOTOR_CRC_H

#include <cstdint>

// Same CRC32 Unitree uses on LowCmd_ (poly 0x04c11db7). Call on the DDS
// struct: crc32_core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1).
uint32_t crc32_core(uint32_t * ptr, uint32_t len);

constexpr int LOWLEVEL = 0xff;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

#endif
