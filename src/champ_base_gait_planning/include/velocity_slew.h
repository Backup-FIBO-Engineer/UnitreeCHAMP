// ROS-free cmd_vel slew + zero-crossing gate used by quadruped_controller_node.
// CHAMP's LegController resets the gait the instant every velocity is exactly
// zero and plants all four feet at stance in that tick. Callers must not pass
// a hard zero while a foot is mid-swing.
#ifndef VELOCITY_SLEW_H
#define VELOCITY_SLEW_H

#include <algorithm>
#include <cmath>

namespace champ_gait
{

inline double slewToward(double current, double target, double max_delta)
{
  if (!(max_delta > 0.0) || !std::isfinite(max_delta) || !std::isfinite(target)) {
    return std::isfinite(target) ? target : current;
  }
  const double delta = target - current;
  if (std::fabs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

struct PlanarVel
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  bool isZero() const
  {
    return vx == 0.0 && vy == 0.0 && wz == 0.0;
  }
};

inline bool allStance(const bool stance[4])
{
  return stance[0] && stance[1] && stance[2] && stance[3];
}

// Slews current toward target at the given accelerations. If the slewed value
// would be exactly zero while the robot is still moving, keep current unless
// allow_zero is true (touchdown or all four feet in stance).
inline PlanarVel slewPlanarVel(
  PlanarVel current,
  PlanarVel target,
  double max_linear_acceleration,
  double max_angular_acceleration,
  double dt,
  bool allow_zero)
{
  const double dv = max_linear_acceleration * dt;
  const double dw = max_angular_acceleration * dt;
  PlanarVel next;
  next.vx = slewToward(current.vx, target.vx, dv);
  next.vy = slewToward(current.vy, target.vy, dv);
  next.wz = slewToward(current.wz, target.wz, dw);
  if (!current.isZero() && next.isZero() && !allow_zero) {
    return current;
  }
  return next;
}

}  // namespace champ_gait

#endif  // VELOCITY_SLEW_H
