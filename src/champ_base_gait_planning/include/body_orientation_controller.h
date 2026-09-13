// IMU-only body orientation loop for CHAMP (no ROS, no robot constants).
//
// CHAMP's BodyController rotates the stance feet opposite to the requested
// body roll/pitch before IK, so on flat ground a requested roll r yields a body
// roll of r. This slow outer loop compares the roll/pitch of the base link
// measured by the IMU with the desired roll/pitch and adds a bounded
// correction to the pose CHAMP receives:
//
//   command = desired + u,   u = kp * e + ki * integral(e) - kd * body_rate,
//   e = desired - measured (low-pass filtered, dead-banded).
//
// The integral term removes the steady-state error a proportional term alone
// would leave (the plant is roughly unit gain: a slope of theta gives a body
// angle of theta + command, so the correction must converge to -theta). The
// gait, IK and joint PD loops are untouched; only the roll/pitch that the
// body controller already accepts is closed on the IMU. Yaw is never taken
// from the IMU (it drifts and CHAMP steers yaw through cmd_vel).
//
// Everything here is dimensionless or in rad / rad/s; the per-robot limits and
// gains come from config/<robot>_body_pose.yaml through the node.
#ifndef BODY_ORIENTATION_CONTROLLER_H
#define BODY_ORIENTATION_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace champ_body_pose
{

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RollPitchYaw
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

inline double clampAbs(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  return std::max(-limit, std::min(limit, value));
}

inline Quaternion normalized(const Quaternion & q)
{
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!(n > 1e-12) || !std::isfinite(n)) {
    return Quaternion{};
  }
  return Quaternion{q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Quaternion conjugate(const Quaternion & q)
{
  return Quaternion{q.w, -q.x, -q.y, -q.z};
}

// Hamilton product a * b: rotate by b first, then by a.
inline Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return Quaternion{
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// v expressed in the parent frame of q (q = parent <- child, v in child).
inline Vector3 rotate(const Quaternion & q, const Vector3 & v)
{
  const Quaternion p{0.0, v.x, v.y, v.z};
  const Quaternion r = multiply(multiply(q, p), conjugate(q));
  return Vector3{r.x, r.y, r.z};
}

// Same convention as tf2::Quaternion::setRPY / urdf::Rotation::setFromRPY:
// R = Rz(yaw) * Ry(pitch) * Rx(roll), i.e. roll about the body X axis.
inline Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return Quaternion{
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy};
}

// Same convention as tf2::Matrix3x3::getRPY (solution 1), which the CHAMP
// quadruped controller uses on the /body_pose quaternion.
inline RollPitchYaw rpyFromQuaternion(const Quaternion & q_in)
{
  const Quaternion q = normalized(q_in);
  const double r20 = 2.0 * (q.x * q.z - q.w * q.y);
  const double r21 = 2.0 * (q.y * q.z + q.w * q.x);
  const double r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double r10 = 2.0 * (q.x * q.y + q.w * q.z);
  const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  RollPitchYaw rpy;
  rpy.roll = std::atan2(r21, r22);
  rpy.pitch = std::atan2(-r20, std::sqrt(r21 * r21 + r22 * r22));
  rpy.yaw = std::atan2(r10, r00);
  return rpy;
}

// Gains shared by the roll and pitch axes.
struct AxisGains
{
  double kp{0.0};                // rad of correction per rad of error
  double ki{0.0};                // rad/s of correction per rad of error
  double kd{0.0};                // rad of correction per rad/s of body rate
  double filter_cutoff_hz{0.0};  // first-order low-pass on angle and rate, <= 0 = off
  double deadband{0.0};          // rad, |error| below it is treated as zero
  double max_correction{0.0};    // rad, |u| bound and integral anti-windup clamp
  double max_rate{0.0};          // rad/s, slew limit of u, <= 0 = none
  double max_error{0.0};         // rad, |error| above it resets u (not standing), <= 0 = off

  void validate() const
  {
    auto nonNegative = [](double v, const char * name) {
        if (!std::isfinite(v) || v < 0.0) {
          throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
        }
      };
    nonNegative(kp, "kp");
    nonNegative(ki, "ki");
    nonNegative(kd, "kd");
    nonNegative(filter_cutoff_hz, "filter_cutoff_hz");
    nonNegative(deadband, "deadband");
    nonNegative(max_rate, "max_rate");
    nonNegative(max_error, "max_error");
    if (!std::isfinite(max_correction) || max_correction <= 0.0) {
      throw std::invalid_argument("max_correction must be finite and > 0");
    }
    if (max_error > 0.0 && max_error <= deadband) {
      throw std::invalid_argument("max_error must be larger than deadband");
    }
  }
};

// One axis (roll or pitch) of the outer loop.
class AxisController
{
public:
  explicit AxisController(const AxisGains & gains = AxisGains{})
  : gains_(gains) {}

  void setGains(const AxisGains & gains)
  {
    gains.validate();
    gains_ = gains;
  }

  const AxisGains & gains() const {return gains_;}

  void reset()
  {
    filter_initialized_ = false;
    angle_f_ = 0.0;
    rate_f_ = 0.0;
    integral_ = 0.0;
    u_ = 0.0;
    error_reset_ = false;
  }

  // One step with a fresh measurement. Returns the correction u (rad).
  double update(double desired, double measured, double rate, double dt)
  {
    if (!(dt > 0.0) || !std::isfinite(dt)) {
      return u_;
    }
    filter(measured, rate, dt);
    double error = desired - angle_f_;
    error_reset_ = gains_.max_error > 0.0 && std::fabs(error) > gains_.max_error;
    if (error_reset_) {
      // Not standing on its feet (fallen, lying, carried): a kinematic body
      // pose cannot fix that and integrating it would wind up. Back off.
      integral_ = 0.0;
      u_ = slew(0.0, dt);
      return u_;
    }
    if (std::fabs(error) < gains_.deadband) {
      error = 0.0;
    }
    integral_ = clampAbs(integral_ + gains_.ki * error * dt, gains_.max_correction);
    const double target = clampAbs(
      gains_.kp * error + integral_ - gains_.kd * rate_f_, gains_.max_correction);
    u_ = slew(target, dt);
    return u_;
  }

  // No usable measurement: slew the correction back to zero and forget the
  // filter/integral state so the next measurement starts clean.
  double relax(double dt)
  {
    filter_initialized_ = false;
    integral_ = 0.0;
    error_reset_ = false;
    if (dt > 0.0 && std::isfinite(dt)) {
      u_ = slew(0.0, dt);
    }
    return u_;
  }

  double correction() const {return u_;}
  double filteredAngle() const {return angle_f_;}
  double filteredRate() const {return rate_f_;}
  double integral() const {return integral_;}
  bool errorReset() const {return error_reset_;}

private:
  void filter(double angle, double rate, double dt)
  {
    if (!filter_initialized_ || gains_.filter_cutoff_hz <= 0.0) {
      angle_f_ = angle;
      rate_f_ = rate;
      filter_initialized_ = true;
      return;
    }
    const double tau = 1.0 / (2.0 * M_PI * gains_.filter_cutoff_hz);
    const double alpha = dt / (tau + dt);
    angle_f_ += alpha * (angle - angle_f_);
    rate_f_ += alpha * (rate - rate_f_);
  }

  double slew(double target, double dt) const
  {
    if (gains_.max_rate <= 0.0) {
      return target;
    }
    const double step = gains_.max_rate * dt;
    return u_ + std::max(-step, std::min(step, target - u_));
  }

  AxisGains gains_;
  bool filter_initialized_{false};
  double angle_f_{0.0};
  double rate_f_{0.0};
  double integral_{0.0};
  double u_{0.0};
  bool error_reset_{false};
};

struct BodyOrientationConfig
{
  AxisGains gains;
  double max_roll{0.0};   // rad, |commanded body roll| (desired + correction)
  double max_pitch{0.0};  // rad, |commanded body pitch|
  // base <- imu rotation (URDF fixed joints between links_map.base and links_map.imu).
  Quaternion base_from_imu{};

  void validate() const
  {
    gains.validate();
    if (!std::isfinite(max_roll) || max_roll <= 0.0) {
      throw std::invalid_argument("max_roll must be finite and > 0");
    }
    if (!std::isfinite(max_pitch) || max_pitch <= 0.0) {
      throw std::invalid_argument("max_pitch must be finite and > 0");
    }
  }
};

// Body roll/pitch of the base link and body-frame angular rates derived from one IMU sample.
struct BodyMeasurement
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double roll_rate{0.0};
  double pitch_rate{0.0};
};

struct BodyCommand
{
  double roll{0.0};              // what CHAMP's body controller receives
  double pitch{0.0};
  double roll_correction{0.0};   // closed-loop share of it
  double pitch_correction{0.0};
  bool error_reset{false};       // measurement further than max_error from desired
};

class BodyOrientationController
{
public:
  // Unconfigured until configure(): every limit is zero, so update() would
  // only ever return zero corrections.
  BodyOrientationController() = default;

  explicit BodyOrientationController(const BodyOrientationConfig & config)
  {
    configure(config);
  }

  void configure(const BodyOrientationConfig & config)
  {
    config.validate();
    config_ = config;
    config_.base_from_imu = normalized(config.base_from_imu);
    roll_.setGains(config.gains);
    pitch_.setGains(config.gains);
  }

  const BodyOrientationConfig & config() const {return config_;}

  void reset()
  {
    roll_.reset();
    pitch_.reset();
  }

  // world <- imu orientation and imu-frame angular velocity -> base roll/pitch + rates.
  BodyMeasurement measure(const Quaternion & world_from_imu, const Vector3 & imu_angular_velocity) const
  {
    const Quaternion world_from_base =
      multiply(normalized(world_from_imu), conjugate(config_.base_from_imu));
    const RollPitchYaw rpy = rpyFromQuaternion(world_from_base);
    const Vector3 rate = rotate(config_.base_from_imu, imu_angular_velocity);
    BodyMeasurement m;
    m.roll = rpy.roll;
    m.pitch = rpy.pitch;
    m.yaw = rpy.yaw;
    m.roll_rate = rate.x;
    m.pitch_rate = rate.y;
    return m;
  }

  BodyCommand update(const RollPitchYaw & desired, const BodyMeasurement & measured, double dt)
  {
    const double u_roll = roll_.update(desired.roll, measured.roll, measured.roll_rate, dt);
    const double u_pitch = pitch_.update(desired.pitch, measured.pitch, measured.pitch_rate, dt);
    return compose(desired, u_roll, u_pitch, roll_.errorReset() || pitch_.errorReset());
  }

  // IMU missing or stale: drift the correction back to zero.
  BodyCommand relax(const RollPitchYaw & desired, double dt)
  {
    return compose(desired, roll_.relax(dt), pitch_.relax(dt), false);
  }

  // Controller off: pass the desired pose through (clamped) and clear state.
  BodyCommand passThrough(const RollPitchYaw & desired)
  {
    reset();
    return compose(desired, 0.0, 0.0, false);
  }

  const AxisController & rollAxis() const {return roll_;}
  const AxisController & pitchAxis() const {return pitch_;}

private:
  BodyCommand compose(const RollPitchYaw & desired, double u_roll, double u_pitch, bool error_reset) const
  {
    BodyCommand cmd;
    cmd.roll_correction = u_roll;
    cmd.pitch_correction = u_pitch;
    cmd.roll = clampAbs(desired.roll + u_roll, config_.max_roll);
    cmd.pitch = clampAbs(desired.pitch + u_pitch, config_.max_pitch);
    cmd.error_reset = error_reset;
    return cmd;
  }

  BodyOrientationConfig config_;
  AxisController roll_;
  AxisController pitch_;
};

}  // namespace champ_body_pose

#endif  // BODY_ORIENTATION_CONTROLLER_H
