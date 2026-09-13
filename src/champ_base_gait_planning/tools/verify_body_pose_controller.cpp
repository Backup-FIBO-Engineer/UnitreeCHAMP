// Offline verification of the IMU body roll/pitch loop for any robot described
// by its URDF + CHAMP yaml files + config/<robot>_body_pose.yaml (no ROS).
//
//   verify_body_pose_controller <urdf> <gait.yaml> <joints.yaml> <links.yaml> <body_pose.yaml>
//
// Checks, with the numbers the node will use:
//   * quaternion <-> roll/pitch/yaw conventions (tf2 getRPY / setRPY),
//     including a real Unitree IMU sample and a non-trivial IMU mounting;
//   * the IMU mount rotation read from the URDF between links_map.base and
//     links_map.imu;
//   * CHAMP's body controller sign: a requested body roll/pitch produces the
//     same body roll/pitch relative to the feet plane, so
//     command = desired + correction(desired - measured) closes with the
//     right sign, and max_roll/max_pitch at once stay inside the IK reach at
//     gait.nominal_height;
//   * max_roll and max_pitch together while walking at the gait yaml maximum
//     velocities: every gait tick reachable by IK and inside the URDF joint
//     limits (plus the kinematic tilt ceilings of the robot, for the record);
//   * the closed loop with this robot's gains on a simple lagged plant:
//     slope compensation, set-point tracking, saturation without wind-up,
//     dead-band, IMU loss release, not-standing release, slew limit,
//     release (not a step) when disabled, desired-pose ramp.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <quadruped_base/quadruped_base.h>

#include "body_orientation_controller.h"
#include "champ_robot_config.h"

using champ_body_pose::AxisGains;
using champ_body_pose::BodyCommand;
using champ_body_pose::BodyMeasurement;
using champ_body_pose::BodyOrientationConfig;
using champ_body_pose::BodyOrientationController;
using champ_body_pose::Quaternion;
using champ_body_pose::RollPitchYaw;
using champ_body_pose::Vector3;

namespace
{
int g_fails = 0;
int g_checks = 0;

void check(bool ok, const std::string & label, const std::string & detail = "")
{
  ++g_checks;
  std::printf(
    "[%s] %s%s%s\n", ok ? "PASS" : "FAIL", label.c_str(),
    detail.empty() ? "" : ": ", detail.c_str());
  if (!ok) {
    ++g_fails;
  }
}

std::string fmt(const char * format, ...)
{
  char buf[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  return std::string(buf);
}

struct BodyPoseParams
{
  BodyOrientationConfig config;
  double control_rate{0.0};
  double imu_timeout_sec{0.0};
  double desired_rate{0.0};
  bool enabled{true};
};

double requireDouble(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("body_pose.") + key + " missing from the body_pose yaml");
  }
  return node[key].as<double>();
}

BodyPoseParams loadBodyPose(const YAML::Node & params)
{
  const YAML::Node bp = params["body_pose"];
  if (!bp || !bp.IsMap()) {
    throw std::runtime_error("body_pose block missing from the body_pose yaml");
  }
  BodyPoseParams p;
  p.config.gains.kp = requireDouble(bp, "kp");
  p.config.gains.ki = requireDouble(bp, "ki");
  p.config.gains.kd = requireDouble(bp, "kd");
  p.config.gains.filter_cutoff_hz = requireDouble(bp, "filter_cutoff_hz");
  p.config.gains.deadband = requireDouble(bp, "deadband");
  p.config.gains.max_correction = requireDouble(bp, "max_correction");
  p.config.gains.max_rate = requireDouble(bp, "max_rate");
  p.config.gains.max_error = requireDouble(bp, "max_error");
  p.config.max_roll = requireDouble(bp, "max_roll");
  p.config.max_pitch = requireDouble(bp, "max_pitch");
  p.control_rate = requireDouble(bp, "control_rate");
  p.imu_timeout_sec = requireDouble(bp, "imu_timeout_sec");
  p.desired_rate = requireDouble(bp, "desired_rate");
  if (bp["enabled"]) {
    p.enabled = bp["enabled"].as<bool>();
  }
  return p;
}

// base <- imu from the URDF: product of the joint rotations from the base down
// to the IMU link (what the node computes with urdf::Model).
Quaternion imuMountFromUrdf(const champ_tools::Urdf & urdf, const std::string & base, const std::string & imu)
{
  Quaternion q;
  std::string current = imu;
  while (current != base) {
    const auto it = urdf.parent_joint_of_link.find(current);
    if (it == urdf.parent_joint_of_link.end()) {
      throw std::runtime_error("links_map.imu '" + imu + "' is not below links_map.base '" + base + "'");
    }
    const champ_tools::UrdfJoint & j = urdf.joints.at(it->second);
    if (j.type != "fixed") {
      throw std::runtime_error("joint '" + j.name + "' between base and IMU is not fixed");
    }
    q = champ_body_pose::multiply(champ_body_pose::quaternionFromRpy(j.rpy.x, j.rpy.y, j.rpy.z), q);
    current = j.parent;
  }
  return q;
}

struct Vec3d
{
  double x, y, z;
};

Vec3d sub(const Vec3d & a, const Vec3d & b) {return Vec3d{a.x - b.x, a.y - b.y, a.z - b.z};}
Vec3d cross(const Vec3d & a, const Vec3d & b)
{
  return Vec3d{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Roll/pitch of the body relative to the plane through the four feet (base
// frame): the plane normal is world-up when the feet stand on flat ground.
RollPitchYaw bodyRpyFromFeet(const Vec3d (&feet)[4])
{
  // CHAMP order LF, RF, LH, RH; average the two triangles' normals.
  Vec3d n1 = cross(sub(feet[1], feet[0]), sub(feet[2], feet[0]));
  Vec3d n2 = cross(sub(feet[2], feet[3]), sub(feet[1], feet[3]));
  if (n1.z < 0.0) {n1 = Vec3d{-n1.x, -n1.y, -n1.z};}
  if (n2.z < 0.0) {n2 = Vec3d{-n2.x, -n2.y, -n2.z};}
  Vec3d n{n1.x + n2.x, n1.y + n2.y, n1.z + n2.z};
  const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  n = Vec3d{n.x / len, n.y / len, n.z / len};
  RollPitchYaw rpy;
  rpy.roll = std::atan2(n.y, n.z);
  rpy.pitch = std::atan2(-n.x, std::sqrt(n.y * n.y + n.z * n.z));
  rpy.yaw = 0.0;
  return rpy;
}

// Body roll/pitch relative to the feet plane for a requested CHAMP body pose,
// through the real pipeline: BodyController -> IK -> FK. Returns false when
// the IK rejects the pose (unreachable).
bool champBodyRpy(
  champ::QuadrupedBase & base, champ::BodyController & body, champ::Kinematics & kin,
  double nominal_height, double roll, double pitch, RollPitchYaw & out, double & fk_err)
{
  champ::Pose pose;
  pose.position.z = static_cast<float>(nominal_height);
  pose.orientation.roll = static_cast<float>(roll);
  pose.orientation.pitch = static_cast<float>(pitch);
  geometry::Transformation feet[4];
  body.poseCommand(feet, pose);
  float q[12];
  for (float & v : q) {
    v = NAN;
  }
  kin.inverse(q, feet);
  for (const float v : q) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  base.updateJointPositions(q);
  Vec3d fk_feet[4];
  fk_err = 0.0;
  for (int i = 0; i < 4; ++i) {
    geometry::Transformation fb = base.legs[i]->foot_from_base();
    fk_feet[i] = Vec3d{fb.X(), fb.Y(), fb.Z()};
    // poseCommand leaves the target in the hip frame; compare in the base frame.
    geometry::Transformation target = feet[i];
    target.Translate(base.legs[i]->hip.x(), base.legs[i]->hip.y(), base.legs[i]->hip.z());
    fk_err = std::max(
      fk_err, std::sqrt(
        std::pow(target.X() - fb.X(), 2) + std::pow(target.Y() - fb.Y(), 2) +
        std::pow(target.Z() - fb.Z(), 2)));
  }
  out = bodyRpyFromFeet(fk_feet);
  return true;
}

// URDF <limit lower upper> of the 12 CHAMP joints, when every joint has one.
struct JointLimits
{
  bool numeric{false};
  float lower[12];
  float upper[12];
};

JointLimits loadLimits(const champ_tools::RobotConfig & cfg)
{
  JointLimits limits;
  limits.numeric = true;
  const std::vector<std::string> names = cfg.champJointNames();
  for (int i = 0; i < 12; ++i) {
    const auto it = cfg.urdf.joints.find(names[static_cast<size_t>(i)]);
    if (it == cfg.urdf.joints.end() || !it->second.has_limits || !(it->second.lower < it->second.upper)) {
      limits.numeric = false;
      limits.lower[i] = -INFINITY;
      limits.upper[i] = INFINITY;
      continue;
    }
    limits.lower[i] = static_cast<float>(it->second.lower);
    limits.upper[i] = static_cast<float>(it->second.upper);
  }
  return limits;
}

float planarReach(const champ::QuadrupedLeg & leg)
{
  const float l1 = std::sqrt(leg.lower_leg.x() * leg.lower_leg.x() + leg.lower_leg.z() * leg.lower_leg.z());
  const float l2 = std::sqrt(leg.foot.x() * leg.foot.x() + leg.foot.z() * leg.foot.z());
  return l1 + l2;
}

struct VelocityCase
{
  std::string name;
  float vx;
  float vy;
  float wz;
};

// Gait ticks (200 Hz) at a body velocity with a requested body roll/pitch
// through BodyController -> LegController -> IK: every tick must be reachable
// (CHAMP's IK discards the whole plan and the legs freeze otherwise) and
// inside the URDF joint limits (the bridge clamps, the foot lands elsewhere).
// Returns an empty string when fine, else what failed first.
std::string gaitWithTilt(
  const champ_tools::RobotConfig & cfg, const champ::GaitConfig & gait_in, const JointLimits & limits,
  double roll, double pitch, const VelocityCase & v, int ticks)
{
  champ::GaitConfig gait = gait_in;  // QuadrupedBase takes a non-const reference
  champ::QuadrupedBase base(gait);
  cfg.applyTo(base);
  champ::Kinematics kin(base);
  champ::BodyController body(base);
  champ::LegController legs(base, 0);
  champ::Pose pose;
  pose.position.z = gait.nominal_height;
  pose.orientation.roll = static_cast<float>(roll);
  pose.orientation.pitch = static_cast<float>(pitch);
  const std::vector<std::string> names = cfg.champJointNames();
  for (int t = 0; t < ticks; ++t) {
    geometry::Transformation feet[4];
    body.poseCommand(feet, pose);
    champ::Velocities cmd;
    cmd.linear.x = v.vx;
    cmd.linear.y = v.vy;
    cmd.angular.z = v.wz;
    legs.velocityCommand(feet, cmd, static_cast<champ::PhaseGenerator::Time>(t) * 5000ul);
    for (int i = 0; i < 4; ++i) {
      const float len = std::sqrt(
        feet[i].X() * feet[i].X() + feet[i].Y() * feet[i].Y() + feet[i].Z() * feet[i].Z());
      if (len >= planarReach(*base.legs[i])) {
        return fmt("leg %d beyond reach at tick %d", i, t);
      }
    }
    float q[12];
    for (float & j : q) {
      j = NAN;
    }
    kin.inverse(q, feet);
    for (int i = 0; i < 12; ++i) {
      if (!std::isfinite(q[i])) {
        return fmt("IK NaN at tick %d", t);
      }
      if (limits.numeric && (q[i] < limits.lower[i] || q[i] > limits.upper[i])) {
        return fmt("%s outside URDF limits at tick %d", names[static_cast<size_t>(i)].c_str(), t);
      }
    }
  }
  return "";
}

// Largest tilt (both signs, 0.005 rad steps) that gaitWithTilt accepts.
double tiltCeiling(
  const champ_tools::RobotConfig & cfg, const champ::GaitConfig & gait, const JointLimits & limits,
  bool roll_axis, bool pitch_axis, const VelocityCase & v, int ticks)
{
  double last_ok = 0.0;
  for (double a = 0.005; a <= 1.5; a += 0.005) {
    const double r = roll_axis ? a : 0.0;
    const double p = pitch_axis ? a : 0.0;
    if (!gaitWithTilt(cfg, gait, limits, r, p, v, ticks).empty() ||
      !gaitWithTilt(cfg, gait, limits, -r, -p, v, ticks).empty())
    {
      return last_ok;
    }
    last_ok = a;
  }
  return last_ok;
}

// First-order plant: the body angle follows the commanded CHAMP body angle
// with a lag (joint PD + body dynamics) on top of the ground slope.
struct Plant
{
  double tilt{0.0};
  double tau{0.15};
  double lagged{0.0};
  double body{0.0};
  double rate{0.0};

  void step(double command, double dt, double disturbance)
  {
    lagged += (command - lagged) * dt / tau;
    const double next = tilt + lagged + disturbance;
    rate = (next - body) / dt;
    body = next;
  }
};

struct LoopResult
{
  double final_error{0.0};
  double final_correction{0.0};
  double max_abs_correction{0.0};
  double max_abs_integral{0.0};
  double max_step{0.0};
  bool any_reset{false};
};

LoopResult runLoop(
  const BodyPoseParams & p, double tilt, double desired_pitch, double seconds, bool disturbance)
{
  BodyOrientationController ctrl(p.config);
  Plant plant;
  plant.tilt = tilt;
  const double dt = 1.0 / p.control_rate;
  RollPitchYaw desired;
  desired.pitch = desired_pitch;
  LoopResult r;
  double previous_u = 0.0;
  double command = desired_pitch;
  const int steps = static_cast<int>(std::lround(seconds / dt));
  for (int i = 0; i < steps; ++i) {
    const double t = i * dt;
    // Trot-like body oscillation seen by the IMU while walking.
    const double d = disturbance ? 0.01 * std::sin(2.0 * M_PI * 2.5 * t) : 0.0;
    plant.step(command, dt, d);
    BodyMeasurement m;
    m.pitch = plant.body;
    m.pitch_rate = plant.rate;
    const BodyCommand cmd = ctrl.update(desired, m, dt);
    command = cmd.pitch;
    r.max_abs_correction = std::max(r.max_abs_correction, std::fabs(cmd.pitch_correction));
    r.max_abs_integral = std::max(r.max_abs_integral, std::fabs(ctrl.pitchAxis().integral()));
    r.max_step = std::max(r.max_step, std::fabs(cmd.pitch_correction - previous_u));
    previous_u = cmd.pitch_correction;
    r.any_reset = r.any_reset || cmd.error_reset;
  }
  r.final_error = desired_pitch - plant.body;
  r.final_correction = previous_u;
  return r;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 6) {
    std::fprintf(
      stderr, "usage: %s <urdf> <gait.yaml> <joints.yaml> <links.yaml> <body_pose.yaml>\n",
      argv[0]);
    return 2;
  }
  champ_tools::RobotConfig cfg;
  BodyPoseParams params;
  try {
    cfg = champ_tools::loadRobotConfig(argv[1], argv[2], argv[3], argv[4]);
    params = loadBodyPose(champ_tools::loadRosParams({argv[5]}));
    params.config.base_from_imu = imuMountFromUrdf(cfg.urdf, cfg.base_link, cfg.imu_link);
    params.config.validate();
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "%s\n", ex.what());
    return 2;
  }
  const AxisGains & g = params.config.gains;
  std::printf(
    "robot %s: kp %.3f ki %.3f kd %.3f cutoff %.1f Hz deadband %.4f |u|<=%.3f slew %.2f "
    "max_roll %.3f max_pitch %.3f max_error %.3f rate %.0f Hz imu timeout %.2f s\n",
    cfg.urdf.name.c_str(), g.kp, g.ki, g.kd, g.filter_cutoff_hz, g.deadband,
    g.max_correction, g.max_rate, params.config.max_roll, params.config.max_pitch,
    g.max_error, params.control_rate, params.imu_timeout_sec);

  // --- quaternion conventions ---------------------------------------------
  {
    double max_err = 0.0;
    for (double roll = -1.2; roll <= 1.2; roll += 0.4) {
      for (double pitch = -1.2; pitch <= 1.2; pitch += 0.4) {
        for (double yaw = -3.0; yaw <= 3.0; yaw += 1.0) {
          const RollPitchYaw back = champ_body_pose::rpyFromQuaternion(
            champ_body_pose::quaternionFromRpy(roll, pitch, yaw));
          max_err = std::max({max_err, std::fabs(back.roll - roll), std::fabs(back.pitch - pitch),
              std::fabs(back.yaw - yaw)});
        }
      }
    }
    check(max_err < 1e-9, "rpy -> quaternion -> rpy round trip", fmt("max err %.2e rad", max_err));

    // Gravity direction in the body frame is an independent way to get roll/pitch.
    const Quaternion sample{0.4533901512622833, -0.004343767650425434, 0.004506191238760948,
      0.8912901878356934};  // real Unitree IMU sample (/dog_imu_raw_aligned)
    const RollPitchYaw rpy = champ_body_pose::rpyFromQuaternion(sample);
    const Vector3 up = champ_body_pose::rotate(champ_body_pose::conjugate(sample), Vector3{0.0, 0.0, 1.0});
    const double roll_g = std::atan2(up.y, up.z);
    const double pitch_g = std::atan2(-up.x, std::sqrt(up.y * up.y + up.z * up.z));
    check(
      std::fabs(rpy.roll - roll_g) < 1e-9 && std::fabs(rpy.pitch - pitch_g) < 1e-9,
      "roll/pitch equal the gravity-vector formulas on a real IMU sample",
      fmt("roll %.4f pitch %.4f yaw %.3f rad", rpy.roll, rpy.pitch, rpy.yaw));
    check(
      std::fabs(rpy.roll) < 0.05 && std::fabs(rpy.pitch) < 0.05 && std::fabs(rpy.yaw) > 2.0,
      "real IMU sample: robot nearly level, yaw large (yaw is never used)");

    const Vector3 v = champ_body_pose::rotate(
      champ_body_pose::quaternionFromRpy(0.0, 0.0, M_PI / 2.0), Vector3{1.0, 0.0, 0.0});
    check(
      std::fabs(v.x) < 1e-9 && std::fabs(v.y - 1.0) < 1e-9 && std::fabs(v.z) < 1e-9,
      "rotate(): yaw +90 deg maps +X to +Y (right-handed)");

    // Right-hand sign: body rolled +0.1 -> IMU quaternion with positive roll.
    const RollPitchYaw pos = champ_body_pose::rpyFromQuaternion(
      champ_body_pose::quaternionFromRpy(0.1, 0.0, 0.0));
    check(pos.roll > 0.099 && pos.roll < 0.101, "positive roll quaternion reads back positive");
  }

  // --- IMU mounting ---------------------------------------------------------
  {
    const RollPitchYaw mount = champ_body_pose::rpyFromQuaternion(params.config.base_from_imu);
    check(
      true, fmt("URDF %s -> %s mount", cfg.base_link.c_str(), cfg.imu_link.c_str()),
      fmt("rpy %.4f %.4f %.4f rad", mount.roll, mount.pitch, mount.yaw));

    // With the robot's mount, an IMU quaternion built from a known base
    // orientation must measure that base orientation.
    BodyOrientationController ctrl(params.config);
    const Quaternion world_from_base = champ_body_pose::quaternionFromRpy(0.10, -0.05, 1.0);
    const Quaternion world_from_imu =
      champ_body_pose::multiply(world_from_base, params.config.base_from_imu);
    const BodyMeasurement m = ctrl.measure(world_from_imu, Vector3{});
    check(
      std::fabs(m.roll - 0.10) < 1e-9 && std::fabs(m.pitch + 0.05) < 1e-9,
      "measure(): base roll/pitch recovered through the URDF mount",
      fmt("roll %.5f pitch %.5f", m.roll, m.pitch));

    // Non-trivial mount (IMU yawed +90 deg on the body): still the base angles,
    // and the IMU X rate becomes the base Y rate.
    BodyOrientationConfig yawed = params.config;
    yawed.base_from_imu = champ_body_pose::quaternionFromRpy(0.0, 0.0, M_PI / 2.0);
    BodyOrientationController ctrl_yawed(yawed);
    const Quaternion wfi = champ_body_pose::multiply(world_from_base, yawed.base_from_imu);
    const BodyMeasurement my = ctrl_yawed.measure(wfi, Vector3{1.0, 0.0, 0.0});
    check(
      std::fabs(my.roll - 0.10) < 1e-9 && std::fabs(my.pitch + 0.05) < 1e-9 &&
      std::fabs(my.roll_rate) < 1e-9 && std::fabs(my.pitch_rate - 1.0) < 1e-9,
      "measure(): 90 deg yawed IMU mount handled (angles and rates)",
      fmt("roll %.4f pitch %.4f rates %.3f %.3f", my.roll, my.pitch, my.roll_rate, my.pitch_rate));
  }

  // --- CHAMP body controller sign and reach for this robot --------------------
  champ::GaitConfig gait = cfg.champGait();
  champ::QuadrupedBase base(gait);
  cfg.applyTo(base);
  champ::BodyController body(base);
  champ::Kinematics kin(base);
  {
    RollPitchYaw level;
    double err = 0.0;
    const bool ok = champBodyRpy(base, body, kin, gait.nominal_height, 0.0, 0.0, level, err);
    check(
      ok && std::fabs(level.roll) < 1e-3 && std::fabs(level.pitch) < 1e-3,
      "zero body pose: feet plane level", fmt("roll %.5f pitch %.5f fk err %.5f m", level.roll, level.pitch, err));

    const double test_roll = 0.5 * params.config.max_roll;
    RollPitchYaw rolled;
    const bool ok_r = champBodyRpy(base, body, kin, gait.nominal_height, test_roll, 0.0, rolled, err);
    check(
      ok_r && std::fabs(rolled.roll - test_roll) < 0.01 && std::fabs(rolled.pitch) < 0.01 && err < 2e-3,
      "CHAMP body roll +r tilts the body +r about the feet plane (IMU sign)",
      fmt("requested %.3f got roll %.4f pitch %.4f fk err %.5f m", test_roll, rolled.roll, rolled.pitch, err));

    const double test_pitch = 0.5 * params.config.max_pitch;
    RollPitchYaw pitched;
    const bool ok_p = champBodyRpy(base, body, kin, gait.nominal_height, 0.0, test_pitch, pitched, err);
    check(
      ok_p && std::fabs(pitched.pitch - test_pitch) < 0.01 && std::fabs(pitched.roll) < 0.01 && err < 2e-3,
      "CHAMP body pitch +p tilts the body +p about the feet plane (IMU sign)",
      fmt("requested %.3f got roll %.4f pitch %.4f fk err %.5f m", test_pitch, pitched.roll, pitched.pitch, err));

    // Both yaml limits at once, all four sign combinations, at nominal height.
    bool all_ok = true;
    double worst = 0.0;
    double worst_err = 0.0;
    for (int sr = -1; sr <= 1; sr += 2) {
      for (int sp = -1; sp <= 1; sp += 2) {
        RollPitchYaw got;
        double e = 0.0;
        const bool ok_lim = champBodyRpy(
          base, body, kin, gait.nominal_height, sr * params.config.max_roll,
          sp * params.config.max_pitch, got, e);
        if (!ok_lim) {
          all_ok = false;
          continue;
        }
        worst = std::max({worst, std::fabs(got.roll - sr * params.config.max_roll),
            std::fabs(got.pitch - sp * params.config.max_pitch)});
        worst_err = std::max(worst_err, e);
      }
    }
    check(
      all_ok && worst < 0.02 && worst_err < 2e-3,
      "max_roll and max_pitch together reachable by IK at nominal_height",
      fmt("worst angle err %.4f rad, fk err %.5f m", worst, worst_err));
  }

  // --- walking with the tilt: gait + IK + URDF joint limits ------------------------
  {
    const JointLimits limits = loadLimits(cfg);
    const float vx = gait.max_linear_velocity_x;
    const float vy = gait.max_linear_velocity_y;
    const float wz = gait.max_angular_velocity_z;
    const std::vector<VelocityCase> cases = {
      {"standing", 0.0f, 0.0f, 0.0f},
      {fmt("vx=%.2f", vx), vx, 0.0f, 0.0f},
      {fmt("vy=%.2f", vy), 0.0f, vy, 0.0f},
      {fmt("wz=%.2f", wz), 0.0f, 0.0f, wz},
      {"vx/2 + 2wz/3", vx / 2.0f, 0.0f, wz * 2.0f / 3.0f},
      {"vx + wz/2", vx, 0.0f, wz / 2.0f},
      {"vx + vy + wz", vx, vy, wz},
    };
    const int ticks = 400;  // 2 s at 200 Hz, several gait cycles
    std::printf(
      "kinematic tilt ceilings (rad, both signs; nominal_height %.3f, joint limits %s):\n",
      gait.nominal_height, limits.numeric ? "from URDF" : "none in URDF");
    for (const VelocityCase & v : cases) {
      const std::string level = gaitWithTilt(cfg, gait, limits, 0.0, 0.0, v, ticks);
      if (!level.empty()) {
        // The gait yaml alone already leaves the reach here; verify_champ_robot
        // reports that (with its own tolerance), it is not a body pose matter.
        std::printf(
          "  %-14s gait alone not reachable (%s): tilt not evaluated\n", v.name.c_str(), level.c_str());
        continue;
      }
      const double roll_only = tiltCeiling(cfg, gait, limits, true, false, v, ticks);
      const double pitch_only = tiltCeiling(cfg, gait, limits, false, true, v, ticks);
      const double both = tiltCeiling(cfg, gait, limits, true, true, v, ticks);
      std::printf(
        "  %-14s roll-only %.3f (%.1f deg)  pitch-only %.3f (%.1f deg)  roll=pitch %.3f (%.1f deg)\n",
        v.name.c_str(), roll_only, roll_only * 180.0 / M_PI, pitch_only, pitch_only * 180.0 / M_PI,
        both, both * 180.0 / M_PI);

      std::string failure;
      for (int sr = -1; sr <= 1 && failure.empty(); sr += 2) {
        for (int sp = -1; sp <= 1 && failure.empty(); sp += 2) {
          failure = gaitWithTilt(
            cfg, gait, limits, sr * params.config.max_roll, sp * params.config.max_pitch, v, ticks);
        }
      }
      check(
        failure.empty(),
        fmt("max_roll + max_pitch (all signs) while %s: IK reachable, inside URDF joint limits",
          v.name.c_str()),
        failure.empty() ? fmt("%d ticks", ticks) : failure);
    }
  }

  // --- yaml consistency -------------------------------------------------------
  check(
    params.control_rate >= 20.0 && params.control_rate <= 1000.0,
    "control_rate between 20 and 1000 Hz (outer loop)", fmt("%.0f Hz", params.control_rate));
  check(
    params.imu_timeout_sec > 2.0 / params.control_rate,
    "imu_timeout_sec longer than two loop periods", fmt("%.3f s", params.imu_timeout_sec));
  check(
    g.max_correction <= params.config.max_roll && g.max_correction <= params.config.max_pitch,
    "max_correction within max_roll / max_pitch (full correction usable at desired 0)");
  check(
    g.max_error > std::max(params.config.max_roll, params.config.max_pitch),
    "max_error above max_roll / max_pitch (a desired tilt from level is not a fall)");
  check(
    g.filter_cutoff_hz > 0.0 && g.filter_cutoff_hz < 0.5 * params.control_rate,
    "filter cutoff below Nyquist of the loop", fmt("%.1f Hz", g.filter_cutoff_hz));
  check(g.ki > 0.0, "ki > 0 (steady-state slope compensation needs the integral)");
  check(
    g.max_rate > 0.0 && params.desired_rate > 0.0,
    "max_rate and desired_rate > 0 (the pose handed to CHAMP never steps)",
    fmt("correction <= %.2f rad/s, desired <= %.2f rad/s", g.max_rate, params.desired_rate));

  // --- closed loop with this robot's gains ----------------------------------------
  {
    const double dt = 1.0 / params.control_rate;
    const double tilt = 0.6 * g.max_correction;
    const LoopResult r = runLoop(params, tilt, 0.0, 5.0, false);
    check(
      std::fabs(r.final_error) < 0.01 && std::fabs(r.final_correction + tilt) < 0.01,
      "slope: body levelled, correction = -tilt",
      fmt("tilt %.3f err %.4f u %.4f", tilt, r.final_error, r.final_correction));
    check(!r.any_reset, "slope: never flagged as not standing");
    check(
      r.max_abs_correction <= g.max_correction + 1e-9,
      "slope: |u| within max_correction", fmt("max |u| %.4f", r.max_abs_correction));
    check(
      r.max_step <= g.max_rate * dt * 1.0001 + 1e-12 || g.max_rate <= 0.0,
      "slope: correction slew within max_rate", fmt("max step %.5f rad/loop", r.max_step));

    const LoopResult walk = runLoop(params, tilt, 0.0, 5.0, true);
    check(
      std::fabs(walk.final_error) < 0.03 && std::fabs(walk.final_correction + tilt) < 0.03,
      "slope while trotting (2.5 Hz body wobble): still levelled within 0.03 rad",
      fmt("err %.4f u %.4f", walk.final_error, walk.final_correction));

    const double want = 0.5 * params.config.max_pitch;
    const LoopResult track = runLoop(params, 0.0, want, 5.0, false);
    check(
      std::fabs(track.final_error) < 0.01,
      "set-point: desired pitch tracked on flat ground", fmt("desired %.3f err %.4f", want, track.final_error));

    const double steep = 2.0 * g.max_correction;
    const LoopResult sat = runLoop(params, steep, 0.0, 5.0, false);
    check(
      std::fabs(sat.final_correction + g.max_correction) < 1e-6 &&
      sat.max_abs_integral <= g.max_correction + 1e-9 &&
      std::fabs(sat.final_error + (steep - g.max_correction)) < 0.01,
      "steep slope: correction saturates at max_correction, integral does not wind up",
      fmt("u %.4f integral %.4f err %.4f", sat.final_correction, sat.max_abs_integral, sat.final_error));

    // Dead-band: an error below it produces no correction.
    {
      BodyOrientationController ctrl(params.config);
      BodyMeasurement m;
      m.pitch = -0.5 * g.deadband;
      BodyCommand cmd;
      for (int i = 0; i < 200; ++i) {
        cmd = ctrl.update(RollPitchYaw{}, m, dt);
      }
      check(
        std::fabs(cmd.pitch_correction) < 1e-12, "dead-band: half-deadband error gives no correction",
        fmt("u %.2e", cmd.pitch_correction));
    }

    // IMU loss: the correction slews back to zero and the state is forgotten.
    // (Constant measurement without a plant: the integral winds to the clamp first.)
    {
      BodyOrientationController ctrl(params.config);
      BodyMeasurement m;
      m.pitch = tilt;
      BodyCommand cmd;
      for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
        cmd = ctrl.update(RollPitchYaw{}, m, dt);
      }
      const double before = cmd.pitch_correction;
      const double release_time = g.max_rate > 0.0 ? std::fabs(before) / g.max_rate + 0.2 : 0.2;
      for (int i = 0; i < static_cast<int>(release_time / dt); ++i) {
        cmd = ctrl.relax(RollPitchYaw{}, dt);
      }
      check(
        std::fabs(before + g.max_correction) < 1e-6 && std::fabs(cmd.pitch_correction) < 1e-9 &&
        std::fabs(ctrl.pitchAxis().integral()) < 1e-12,
        "IMU loss: correction released to zero, integral cleared",
        fmt("u before %.4f after %.2e (%.2f s)", before, cmd.pitch_correction, release_time));
    }

    // Not standing: a huge error resets the correction instead of integrating it.
    {
      BodyOrientationController ctrl(params.config);
      BodyMeasurement m;
      m.pitch = tilt;
      BodyCommand cmd;
      for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
        cmd = ctrl.update(RollPitchYaw{}, m, dt);
      }
      m.pitch = g.max_error + 0.1;
      // The low-pass needs a few periods to reach the new value.
      const double settle = 3.0 / (2.0 * M_PI * std::max(g.filter_cutoff_hz, 0.1)) + 1.0 + g.max_correction / std::max(g.max_rate, 1e-3);
      bool flagged = false;
      for (int i = 0; i < static_cast<int>(settle / dt); ++i) {
        cmd = ctrl.update(RollPitchYaw{}, m, dt);
        flagged = flagged || cmd.error_reset;
      }
      check(
        flagged && cmd.error_reset && std::fabs(cmd.pitch_correction) < 1e-9 &&
        std::fabs(ctrl.pitchAxis().integral()) < 1e-12,
        "not standing: error beyond max_error releases the correction",
        fmt("u %.2e reset %d", cmd.pitch_correction, static_cast<int>(cmd.error_reset)));
    }

    // Total clamp: desired at the limit plus a correction never exceeds max_pitch.
    {
      BodyOrientationController ctrl(params.config);
      RollPitchYaw desired;
      desired.pitch = params.config.max_pitch;
      BodyMeasurement m;
      m.pitch = 0.0;
      BodyCommand cmd;
      for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
        cmd = ctrl.update(desired, m, dt);
      }
      check(
        std::fabs(cmd.pitch) <= params.config.max_pitch + 1e-12 && cmd.pitch_correction > 0.0,
        "command clamped to max_pitch even with a positive correction",
        fmt("cmd %.4f u %.4f", cmd.pitch, cmd.pitch_correction));
    }

    // Disabled at runtime while holding a correction: released along max_rate
    // (no step), then the desired pose passes through unchanged.
    {
      BodyOrientationController ctrl(params.config);
      BodyMeasurement m;
      m.pitch = tilt;
      BodyCommand cmd;
      for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
        cmd = ctrl.update(RollPitchYaw{}, m, dt);
      }
      RollPitchYaw desired;
      desired.roll = 0.05;
      desired.pitch = -0.03;
      double previous = cmd.pitch_correction;
      double max_step = 0.0;
      int loops_to_zero = -1;
      for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
        cmd = ctrl.relax(desired, dt);
        max_step = std::max(max_step, std::fabs(cmd.pitch_correction - previous));
        previous = cmd.pitch_correction;
        if (loops_to_zero < 0 && ctrl.released()) {
          loops_to_zero = i + 1;
        }
      }
      check(
        loops_to_zero > 1 && max_step <= g.max_rate * dt * 1.0001 + 1e-12 &&
        std::fabs(cmd.roll - 0.05) < 1e-12 && std::fabs(cmd.pitch + 0.03) < 1e-12 &&
        cmd.roll_correction == 0.0 && cmd.pitch_correction == 0.0,
        "disabled: correction released within max_rate, then desired passes through unchanged",
        fmt("released after %d loops (%.2f s), max step %.5f rad/loop", loops_to_zero,
          loops_to_zero * dt, max_step));
    }

    // Desired pose slew (node side): a step to the limit is ramped at desired_rate
    // and yaw takes the short way around +-pi.
    {
      const double step = params.desired_rate * dt;
      double value = 0.0;
      int loops = 0;
      double max_delta = 0.0;
      while (std::fabs(value - params.config.max_pitch) > 1e-12 && loops < 100000) {
        const double next = champ_body_pose::slewAngle(value, params.config.max_pitch, step);
        max_delta = std::max(max_delta, std::fabs(next - value));
        value = next;
        ++loops;
      }
      const double expected_s = params.config.max_pitch / params.desired_rate;
      // 3.1 -> -3.1 is 0.083 rad the short way (through +-pi), not -6.2.
      const double yaw = champ_body_pose::slewAngle(3.1, -3.1, 0.05);
      const double yaw_moved = champ_body_pose::wrapAngle(yaw - 3.1);
      check(
        max_delta <= step + 1e-12 && std::fabs(loops * dt - expected_s) <= dt + 1e-9 &&
        std::fabs(yaw_moved - 0.05) < 1e-9,
        "desired step ramped at desired_rate; yaw slews the short way across +-pi",
        fmt("0 -> %.3f rad in %.2f s (expected %.2f s), yaw 3.1 -> -3.1 first step %+.3f",
          params.config.max_pitch, loops * dt, expected_s, yaw_moved));
    }
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
