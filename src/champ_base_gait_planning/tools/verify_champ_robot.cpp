// Offline verification of CHAMP IK/FK, gait, and odometry for any robot
// described by its URDF and CHAMP yaml files. Compiles against champ headers,
// tinyxml2 and yaml-cpp only (no ROS).
//
//   verify_champ_robot <urdf> <gait.yaml> <joints.yaml> <links.yaml>
//
// The leg translations are computed exactly like champ::URDF::loadFromString
// (sum of joint origin xyz along links_map), so every number checked here is
// the number the nodes will use.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <odometry/odometry.h>
#include <quadruped_base/quadruped_base.h>

#include "champ_robot_config.h"

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

float wrap_pi(float a)
{
  while (a > M_PI) {
    a -= 2.0f * static_cast<float>(M_PI);
  }
  while (a < -M_PI) {
    a += 2.0f * static_cast<float>(M_PI);
  }
  return a;
}

float hypot3(float x, float y, float z)
{
  return std::sqrt(x * x + y * y + z * z);
}

void rotate_x(float & y, float & z, float phi)
{
  const float cy = std::cos(phi) * y - std::sin(phi) * z;
  const float cz = std::sin(phi) * y + std::cos(phi) * z;
  y = cy;
  z = cz;
}

void rotate_y(float & x, float & z, float theta)
{
  const float cx = std::cos(theta) * x + std::sin(theta) * z;
  const float cz = -std::sin(theta) * x + std::cos(theta) * z;
  x = cx;
  z = cz;
}

// Independent URDF FK: T_hip * Rx(hip) * T_upper * Ry(upper) * T_lower * Ry(lower) * T_foot
// with rpy=0 origins. Must match QuadrupedLeg::foot_from_hip / foot_from_base.
void independent_fk_hip(champ::QuadrupedLeg & leg, float & x, float & y, float & z)
{
  x = leg.foot.x();
  y = leg.foot.y();
  z = leg.foot.z();
  rotate_y(x, z, leg.lower_leg.theta());
  x += leg.lower_leg.x();
  y += leg.lower_leg.y();
  z += leg.lower_leg.z();
  rotate_y(x, z, leg.upper_leg.theta());
  x += leg.upper_leg.x();
  y += leg.upper_leg.y();
  z += leg.upper_leg.z();
}

void independent_fk_base(champ::QuadrupedLeg & leg, float & x, float & y, float & z)
{
  independent_fk_hip(leg, x, y, z);
  rotate_x(y, z, leg.hip.theta());
  x += leg.hip.x();
  y += leg.hip.y();
  z += leg.hip.z();
}

float planar_reach(const champ::QuadrupedLeg & leg)
{
  const float l1 = std::sqrt(
    leg.lower_leg.x() * leg.lower_leg.x() + leg.lower_leg.z() * leg.lower_leg.z());
  const float l2 = std::sqrt(
    leg.foot.x() * leg.foot.x() + leg.foot.z() * leg.foot.z());
  return l1 + l2;
}

bool joints_finite(const float joints[12])
{
  for (int i = 0; i < 12; ++i) {
    if (!std::isfinite(joints[i])) {
      return false;
    }
  }
  return true;
}

void copy_joints(float dst[12], const float src[12])
{
  std::memcpy(dst, src, sizeof(float) * 12);
}

struct JointLimits
{
  bool numeric{false};
  float lower[12];
  float upper[12];
};

// URDF <limit lower upper> of the 12 CHAMP joints, if every joint has one.
JointLimits load_limits(const champ_tools::RobotConfig & cfg)
{
  JointLimits limits;
  limits.numeric = true;
  const std::vector<std::string> names = cfg.champJointNames();
  for (int i = 0; i < 12; ++i) {
    const auto it = cfg.urdf.joints.find(names[static_cast<size_t>(i)]);
    if (it == cfg.urdf.joints.end() || !it->second.has_limits ||
      !(it->second.lower < it->second.upper))
    {
      limits.numeric = false;
      limits.lower[i] = -std::numeric_limits<float>::infinity();
      limits.upper[i] = std::numeric_limits<float>::infinity();
      continue;
    }
    limits.lower[i] = static_cast<float>(it->second.lower);
    limits.upper[i] = static_cast<float>(it->second.upper);
  }
  return limits;
}

int count_limit_violations(const JointLimits & limits, const float q[12], float margin)
{
  int n = 0;
  for (int i = 0; i < 12; ++i) {
    if (q[i] < limits.lower[i] - margin || q[i] > limits.upper[i] + margin) {
      ++n;
    }
  }
  return n;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::fprintf(
      stderr, "usage: %s <urdf> <gait.yaml> <joints.yaml> <links.yaml>\n", argv[0]);
    return 2;
  }
  champ_tools::RobotConfig cfg;
  try {
    cfg = champ_tools::loadRobotConfig(argv[1], argv[2], argv[3], argv[4]);
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "%s\n", ex.what());
    return 2;
  }
  std::printf(
    "Robot '%s' (root %s, base %s, imu %s), nominal_height %.3f\n",
    cfg.urdf.name.c_str(), cfg.urdf.root.c_str(), cfg.base_link.c_str(),
    cfg.imu_link.c_str(), cfg.gait.nominal_height);

  champ::GaitConfig gait = cfg.champGait();
  champ::QuadrupedBase base(gait);
  cfg.applyTo(base);
  const JointLimits limits = load_limits(cfg);

  // --- URDF contract: CHAMP IK assumes hip=X, thigh/calf=Y, rpy=0 on the leg chain ---
  for (int leg = 0; leg < champ_tools::kLegCount; ++leg) {
    const champ_tools::LegChain & chain = cfg.joints[static_cast<size_t>(leg)];
    const champ_tools::LegChain & links = cfg.links[static_cast<size_t>(leg)];
    for (int i = 0; i < champ_tools::kChainLength; ++i) {
      const auto it = cfg.urdf.joints.find(chain[static_cast<size_t>(i)]);
      if (it == cfg.urdf.joints.end()) {
        check(false, fmt("%s joint '%s' exists in URDF", champ_tools::kLegKeys[leg],
          chain[static_cast<size_t>(i)].c_str()));
        continue;
      }
      const champ_tools::UrdfJoint & j = it->second;
      check(
        j.child == links[static_cast<size_t>(i)],
        fmt("%s joints_map[%d] drives links_map[%d]", champ_tools::kLegKeys[leg], i, i),
        fmt("joint %s child=%s links_map=%s", j.name.c_str(), j.child.c_str(),
          links[static_cast<size_t>(i)].c_str()));
      check(
        std::fabs(j.rpy.x) < 1e-9 && std::fabs(j.rpy.y) < 1e-9 && std::fabs(j.rpy.z) < 1e-9,
        fmt("%s rpy=0 (CHAMP sums xyz only)", j.name.c_str()),
        fmt("%g %g %g", j.rpy.x, j.rpy.y, j.rpy.z));
      if (i < champ_tools::kJointsPerLeg) {
        const bool want_x = i == 0;
        const bool ok = want_x ?
          (std::fabs(j.axis.x - 1.0) < 1e-9 && std::fabs(j.axis.y) < 1e-9 && std::fabs(j.axis.z) < 1e-9) :
          (std::fabs(j.axis.x) < 1e-9 && std::fabs(j.axis.y - 1.0) < 1e-9 && std::fabs(j.axis.z) < 1e-9);
        check(
          (j.type == "revolute" || j.type == "continuous") && ok,
          fmt("%s is revolute about %s", j.name.c_str(), want_x ? "+X" : "+Y"),
          fmt("type=%s axis=%g %g %g", j.type.c_str(), j.axis.x, j.axis.y, j.axis.z));
      } else {
        check(j.type == "fixed", fmt("%s foot joint is fixed", j.name.c_str()), j.type);
      }
    }
    const auto hip = cfg.urdf.joints.find(chain[0]);
    if (hip != cfg.urdf.joints.end()) {
      check(
        hip->second.parent == cfg.base_link && cfg.base_link == cfg.urdf.root,
        fmt("%s hip joint hangs off links_map.base which is the URDF root",
          champ_tools::kLegKeys[leg]),
        fmt("parent=%s base=%s root=%s", hip->second.parent.c_str(), cfg.base_link.c_str(),
          cfg.urdf.root.c_str()));
    }
  }
  check(
    !cfg.imu_link.empty() && cfg.urdf.hasLink(cfg.imu_link),
    "links_map.imu names a URDF link", cfg.imu_link);

  // --- Geometry derived from the URDF chain ---
  const float lf_reach = planar_reach(base.lf);
  check(lf_reach > 0.0f, "LF planar reach l1+l2 from URDF", fmt("%.4f m", lf_reach));
  float reach_spread = 0.0f;
  for (int i = 1; i < 4; ++i) {
    reach_spread = std::max(reach_spread, std::fabs(planar_reach(*base.legs[i]) - lf_reach));
  }
  check(reach_spread < 1e-3f, "all legs have the same planar reach", fmt("spread %.5f m", reach_spread));

  for (int i = 0; i < 4; ++i) {
    const champ::QuadrupedLeg & leg = *base.legs[i];
    const float l0 = leg.upper_leg.y() + leg.lower_leg.y() + leg.foot.y();
    const bool outward = (leg.hip.y() > 0.0f && l0 > 0.0f) || (leg.hip.y() < 0.0f && l0 < 0.0f);
    check(
      outward && std::fabs(l0) > 1e-3f,
      fmt("%s l0 points outward like the hip", champ_tools::kLegKeys[i]),
      fmt("hip.y=%.4f l0=%.5f", leg.hip.y(), l0));
  }
  // Hip layout is informational: CHAMP solves each leg in its own hip frame, so an
  // asymmetric body (CAD export offsets) is legitimate.
  std::printf(
    "[INFO] hip xyz LF(%.4f %.4f %.4f) RF(%.4f %.4f %.4f) LH(%.4f %.4f %.4f) RH(%.4f %.4f %.4f)\n",
    base.lf.hip.x(), base.lf.hip.y(), base.lf.hip.z(), base.rf.hip.x(), base.rf.hip.y(),
    base.rf.hip.z(), base.lh.hip.x(), base.lh.hip.y(), base.lh.hip.z(), base.rh.hip.x(),
    base.rh.hip.y(), base.rh.hip.z());
  check(
    base.lf.hip.x() > 0.0f && base.rf.hip.x() > 0.0f && base.lh.hip.x() < 0.0f &&
    base.rh.hip.x() < 0.0f,
    "front hips ahead of hind hips (+X forward)");
  check(
    base.lf.hip.y() > 0.0f && base.lh.hip.y() > 0.0f && base.rf.hip.y() < 0.0f &&
    base.rh.hip.y() < 0.0f,
    "left hips at +Y, right hips at -Y");

  geometry::Transformation zs = base.lf.zero_stance();
  const float zs_z_expected =
    base.lf.hip.z() + base.lf.upper_leg.z() + base.lf.lower_leg.z() + base.lf.foot.z();
  check(
    std::fabs(zs.Z() - zs_z_expected) < 1e-6f, "LF zero_stance Z from URDF xyz",
    fmt("Z=%.4f expected %.4f", zs.Z(), zs_z_expected));

  const float stand_z = -(zs.Z() + gait.nominal_height);
  check(
    stand_z > 0.0f && gait.nominal_height < lf_reach,
    "nominal_height is shorter than the stretched leg",
    fmt("nominal %.3f, reach %.3f (%.1f mm of margin)", gait.nominal_height, lf_reach,
      (lf_reach - gait.nominal_height) * 1000.0f));

  champ::Kinematics kinematics(base);
  champ::BodyController body_controller(base);
  champ::LegController leg_controller(base, 0);

  // --- Standing pose: body pose -> IK -> FK should recover hip-frame target ---
  champ::Pose req_pose;
  req_pose.position.z = gait.nominal_height;

  geometry::Transformation stand_feet[4];
  body_controller.poseCommand(stand_feet, req_pose);

  float stand_joints[12];
  for (int i = 0; i < 12; ++i) {
    stand_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  kinematics.inverse(stand_joints, stand_feet);
  check(joints_finite(stand_joints), "standing IK produced 12 finite joints");
  if (!joints_finite(stand_joints)) {
    std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
    return 1;
  }
  if (limits.numeric) {
    check(
      count_limit_violations(limits, stand_joints, 0.0f) == 0,
      "standing pose is inside the URDF joint limits");
  } else {
    std::printf("[INFO] URDF joint limits are not all numeric; limit checks skipped\n");
  }

  base.updateJointPositions(stand_joints);
  float stand_fk_err = 0.0f;
  for (int i = 0; i < 4; ++i) {
    geometry::Transformation fk = base.legs[i]->foot_from_hip();
    stand_fk_err = std::max(
      stand_fk_err,
      hypot3(fk.X() - stand_feet[i].X(), fk.Y() - stand_feet[i].Y(), fk.Z() - stand_feet[i].Z()));
  }
  check(stand_fk_err < 1e-3f, "standing IK/FK roundtrip in hip frame",
    fmt("max |FK-target|=%.6f m", stand_fk_err));

  float height_err = 0.0f;
  for (int i = 0; i < 4; ++i) {
    geometry::Transformation fb = base.legs[i]->foot_from_base();
    height_err = std::max(height_err, std::fabs(fb.Z() + gait.nominal_height));
  }
  check(height_err < 5e-3f, "standing feet at nominal_height in base frame",
    fmt("max |foot_z + nominal|=%.5f m", height_err));

  float indep_err = 0.0f;
  for (int i = 0; i < 4; ++i) {
    float hx, hy, hz, bx, by, bz;
    independent_fk_hip(*base.legs[i], hx, hy, hz);
    independent_fk_base(*base.legs[i], bx, by, bz);
    geometry::Transformation hip_fk = base.legs[i]->foot_from_hip();
    geometry::Transformation base_fk = base.legs[i]->foot_from_base();
    indep_err = std::max(indep_err, hypot3(hx - hip_fk.X(), hy - hip_fk.Y(), hz - hip_fk.Z()));
    indep_err = std::max(indep_err, hypot3(bx - base_fk.X(), by - base_fk.Y(), bz - base_fk.Z()));
  }
  check(indep_err < 1e-6f, "CHAMP FK matches independent Rx/Ry URDF FK at stand",
    fmt("max |champ-independent|=%.7f m", indep_err));

  // --- Node ctor order: Odometry is built before URDF translations are loaded ---
  {
    champ::GaitConfig gait_early = cfg.champGait();
    champ::QuadrupedBase early_base(gait_early);
    champ::Odometry early_odom(early_base, 0);
    cfg.applyTo(early_base);
    early_base.updateJointPositions(stand_joints);
    for (int i = 0; i < 4; ++i) {
      early_base.legs[i]->in_contact(true);
    }
    float first_walk_vx = 0.0f;
    for (int t = 1; t <= 12; ++t) {
      champ::Velocities est;
      const champ::Odometry::Time now_us = static_cast<champ::Odometry::Time>(t) * 20000ul;
      if (t == 12) {
        early_base.lf.in_contact(true);
        early_base.rf.in_contact(false);
        early_base.lh.in_contact(false);
        early_base.rh.in_contact(true);
      }
      early_odom.getVelocities(est, now_us);
      if (t < 12) {
        if (std::fabs(est.linear.x) > 1e-6f || std::fabs(est.linear.y) > 1e-6f) {
          first_walk_vx = est.linear.x;
        }
      } else {
        first_walk_vx = est.linear.x;
      }
    }
    check(std::fabs(first_walk_vx) < 1.0f, "odom after stand does not spike from pre-URDF zeros",
      fmt("first trot vx=%.3f m/s (spike would be several m/s)", first_walk_vx));
  }

  // --- Random reachable targets: FK(q) -> IK -> FK(q') ---
  float max_repro_err = 0.0f;
  int random_ok = 0;
  const int kRandomN = 80;
  for (int n = 0; n < kRandomN; ++n) {
    float q[12];
    for (int j = 0; j < 12; ++j) {
      const float mag = ((j % 3) == 0) ? 0.25f : 0.20f;
      const float t = static_cast<float>((n * 12 + j) % 17) / 16.0f;
      q[j] = stand_joints[j] + mag * (t - 0.5f);
    }
    base.updateJointPositions(q);
    geometry::Transformation feet[4];
    for (int i = 0; i < 4; ++i) {
      feet[i] = base.legs[i]->foot_from_hip();
    }
    float q_ik[12];
    for (int i = 0; i < 12; ++i) {
      q_ik[i] = std::numeric_limits<float>::quiet_NaN();
    }
    kinematics.inverse(q_ik, feet);
    if (!joints_finite(q_ik)) {
      continue;
    }
    base.updateJointPositions(q_ik);
    float err = 0.0f;
    float indep_sample = 0.0f;
    for (int i = 0; i < 4; ++i) {
      geometry::Transformation fk = base.legs[i]->foot_from_hip();
      err = std::max(err, hypot3(fk.X() - feet[i].X(), fk.Y() - feet[i].Y(), fk.Z() - feet[i].Z()));
      float hx, hy, hz;
      independent_fk_hip(*base.legs[i], hx, hy, hz);
      indep_sample = std::max(indep_sample, hypot3(hx - fk.X(), hy - fk.Y(), hz - fk.Z()));
    }
    max_repro_err = std::max(max_repro_err, err);
    if (err < 2e-3f && indep_sample < 1e-6f) {
      ++random_ok;
    }
  }
  check(random_ok >= kRandomN - 5, "IK reproduces FK samples near standing",
    fmt("%d/%d repro <2mm, max err=%.5f m", random_ok, kRandomN, max_repro_err));

  // --- Unreachable target: inverse must not publish garbage joints ---
  geometry::Transformation far_feet[4];
  body_controller.poseCommand(far_feet, req_pose);
  far_feet[0].Z() -= lf_reach;  // |target| > l1 + l2
  float before[12];
  copy_joints(before, stand_joints);
  float after[12];
  copy_joints(after, stand_joints);
  kinematics.inverse(after, far_feet);
  bool unchanged = true;
  bool any_nan = false;
  for (int i = 0; i < 12; ++i) {
    if (std::isnan(after[i])) {
      any_nan = true;
    }
    if (after[i] != before[i]) {
      unchanged = false;
    }
  }
  // Documented CHAMP behavior: if any leg is NaN, the whole plan is discarded
  // (joint_positions unchanged). If a leg is unreachable without writing NaN,
  // uninitialized stack values can leak into the command.
  check(unchanged || any_nan, "unreachable IK does not publish garbage",
    fmt("unchanged=%d any_nan=%d", static_cast<int>(unchanged), static_cast<int>(any_nan)));

  // --- Gait: standing (v=0) keeps all four contacts and standing joints ---
  base.updateJointPositions(stand_joints);
  geometry::Transformation gait_feet[4];
  body_controller.poseCommand(gait_feet, req_pose);
  champ::Velocities vel;
  leg_controller.velocityCommand(gait_feet, vel, 0);
  int contacts = 0;
  for (int i = 0; i < 4; ++i) {
    contacts += base.legs[i]->gait_phase() ? 1 : 0;
  }
  check(contacts == 4, "zero velocity: all four legs in stance phase");

  float still_joints[12];
  for (int i = 0; i < 12; ++i) {
    still_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  kinematics.inverse(still_joints, gait_feet);
  float still_err = 0.0f;
  for (int i = 0; i < 12; ++i) {
    still_err = std::max(still_err, std::fabs(wrap_pi(still_joints[i] - stand_joints[i])));
  }
  check(still_err < 1e-3f, "zero velocity: IK matches standing pose", fmt("max |dq|=%.5f rad", still_err));

  // --- Gait walk at the yaml velocity limits: IK finite, within reach, trot pairing ---
  struct Case
  {
    std::string name;
    float vx;
    float vy;
    float wz;
  };
  const float max_vx = gait.max_linear_velocity_x;
  const float max_vy = gait.max_linear_velocity_y;
  const float max_wz = gait.max_angular_velocity_z;
  const std::vector<Case> cases = {
    {fmt("vx=%.2f", max_vx), max_vx, 0.0f, 0.0f},
    {fmt("vy=%.2f", max_vy), 0.0f, max_vy, 0.0f},
    {fmt("wz=%.2f", max_wz), 0.0f, 0.0f, max_wz},
    {"vx/2 + 2wz/3", max_vx / 2.0f, 0.0f, max_wz * 2.0f / 3.0f},
    {"worst yaml (vx, vy, wz)", max_vx, max_vy, max_wz},
  };

  for (const Case & c : cases) {
    champ::QuadrupedBase walk_base(gait);
    cfg.applyTo(walk_base);
    champ::Kinematics walk_kin(walk_base);
    champ::BodyController walk_body(walk_base);
    champ::LegController walk_legs(walk_base, 0);

    int ik_fail = 0;
    int reach_fail = 0;
    int limit_fail = 0;
    int ticks = 0;
    int trot_ok = 0;
    float first_swing_lift = -1.0f;
    geometry::Transformation stand_ref[4];
    walk_body.poseCommand(stand_ref, req_pose);
    const int kTicks = 400;  // 2 s at 200 Hz
    for (int t = 0; t < kTicks; ++t) {
      const champ::PhaseGenerator::Time now_us = static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      walk_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.linear.x = c.vx;
      cmd.linear.y = c.vy;
      cmd.angular.z = c.wz;
      walk_legs.velocityCommand(feet, cmd, now_us);

      if (first_swing_lift < 0.0f) {
        for (int i = 0; i < 4; ++i) {
          if (!walk_base.legs[i]->gait_phase()) {
            // Standing foot Z is below the base; swing adds a positive lift.
            first_swing_lift = std::max(0.0f, feet[i].Z() - stand_ref[i].Z());
            break;
          }
        }
      }

      for (int i = 0; i < 4; ++i) {
        if (hypot3(feet[i].X(), feet[i].Y(), feet[i].Z()) > planar_reach(*walk_base.legs[i]) + 0.02f) {
          ++reach_fail;
        }
      }

      // Trot: LF/RH share phase, RF/LH opposite (after startup swing lock).
      if (walk_legs.phase_generator.has_started) {
        const bool lf = walk_base.lf.gait_phase();
        const bool rf = walk_base.rf.gait_phase();
        const bool lh = walk_base.lh.gait_phase();
        const bool rh = walk_base.rh.gait_phase();
        if (lf == rh && rf == lh && lf != rf) {
          ++trot_ok;
        }
      }

      float q[12];
      for (int i = 0; i < 12; ++i) {
        q[i] = std::numeric_limits<float>::quiet_NaN();
      }
      walk_kin.inverse(q, feet);
      if (!joints_finite(q)) {
        ++ik_fail;
      } else if (limits.numeric && count_limit_violations(limits, q, 0.0f) > 0) {
        ++limit_fail;
      }
      ++ticks;
    }
    const std::string detail = fmt(
      "ik_fail=%d/%d reach_fail=%d limit_fail=%d trot_diag=%d/%d",
      ik_fail, ticks, reach_fail, limit_fail, trot_ok, ticks);
    check(ik_fail == 0 && reach_fail == 0, c.name, detail);
    check(trot_ok > ticks / 2, c.name + " trot LF-RH / RF-LH", detail);
    check(
      first_swing_lift >= 0.0f && first_swing_lift < 0.5f * gait.swing_height + 0.005f,
      c.name + " first swing leaves the ground (not mid-air at swing_height)",
      fmt("first lift %.4f m, swing_height %.4f m", first_swing_lift, gait.swing_height));
    if (limits.numeric) {
      check(limit_fail == 0, c.name + " inside URDF joint limits", detail);
    }
  }

  // --- Odometry: open-loop planned joints + gait_phase contacts, straight walk ---
  {
    champ::QuadrupedBase odom_base(gait);
    cfg.applyTo(odom_base);
    champ::Kinematics odom_kin(odom_base);
    champ::BodyController odom_body(odom_base);
    champ::LegController odom_legs(odom_base, 0);
    champ::Odometry odom(odom_base, 0);

    const float cmd_vx = 0.4f * max_vx;
    const int ctrl_hz = 200;
    const int odom_hz = 50;
    const int seconds = 2;
    const int ctrl_ticks = ctrl_hz * seconds;
    float last_q[12] = {};
    kinematics.inverse(last_q, stand_feet);

    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double sum_wz = 0.0;
    int odom_samples = 0;
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    const float odom_dt = 1.0f / static_cast<float>(odom_hz);

    for (int t = 0; t < ctrl_ticks; ++t) {
      const champ::PhaseGenerator::Time now_us = static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      odom_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.linear.x = cmd_vx;
      odom_legs.velocityCommand(feet, cmd, now_us);
      float q[12];
      copy_joints(q, last_q);
      odom_kin.inverse(q, feet);
      if (joints_finite(q)) {
        copy_joints(last_q, q);
      }
      odom_base.updateJointPositions(last_q);
      for (int i = 0; i < 4; ++i) {
        odom_base.legs[i]->in_contact(odom_base.legs[i]->gait_phase());
      }

      if (t % (ctrl_hz / odom_hz) == 0) {
        champ::Velocities est;
        odom.getVelocities(est, now_us);
        sum_vx += est.linear.x;
        sum_vy += est.linear.y;
        sum_wz += est.angular.z;
        ++odom_samples;
        x += (est.linear.x * std::cos(yaw) - est.linear.y * std::sin(yaw)) * odom_dt;
        y += (est.linear.x * std::sin(yaw) + est.linear.y * std::cos(yaw)) * odom_dt;
        yaw += est.angular.z * odom_dt;
      }
    }

    const float mean_vx = static_cast<float>(sum_vx / std::max(1, odom_samples));
    const float mean_vy = static_cast<float>(sum_vy / std::max(1, odom_samples));
    const float mean_wz = static_cast<float>(sum_wz / std::max(1, odom_samples));
    const std::string detail = fmt(
      "mean vx=%.3f (cmd %.3f * scaler %.2f => %.3f) vy=%.3f wz=%.3f  integrated x=%.3f m in %ds",
      mean_vx, cmd_vx, gait.odom_scaler, cmd_vx * gait.odom_scaler, mean_vy, mean_wz, x, seconds);
    // Generous band: CHAMP odom is low-pass (beta=0.1) and zeros when 4-foot contact.
    check(mean_vx > 0.4f * cmd_vx && mean_vx < 1.4f * cmd_vx,
      "odom vx has correct sign and order of magnitude", detail);
    check(std::fabs(mean_vy) < 0.25f * cmd_vx, "odom vy near 0 for straight walk", detail);
    check(std::fabs(mean_wz) < 0.15f, "odom wz near 0 for straight walk", detail);
    check(x > 0.25f * cmd_vx * seconds, "integrated /odom/raw x advances forward", detail);
  }

  // --- Odometry: yaw in place ---
  {
    champ::QuadrupedBase odom_base(gait);
    cfg.applyTo(odom_base);
    champ::Kinematics odom_kin(odom_base);
    champ::BodyController odom_body(odom_base);
    champ::LegController odom_legs(odom_base, 0);
    champ::Odometry odom(odom_base, 0);

    const float cmd_wz = 0.8f * max_wz;
    const int ctrl_hz = 200;
    const int odom_hz = 50;
    const int seconds = 2;
    const int ctrl_ticks = ctrl_hz * seconds;
    float last_q[12] = {};
    copy_joints(last_q, stand_joints);

    double sum_wz = 0.0;
    int odom_samples = 0;
    for (int t = 0; t < ctrl_ticks; ++t) {
      const champ::PhaseGenerator::Time now_us = static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      odom_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.angular.z = cmd_wz;
      odom_legs.velocityCommand(feet, cmd, now_us);
      float q[12];
      copy_joints(q, last_q);
      odom_kin.inverse(q, feet);
      if (joints_finite(q)) {
        copy_joints(last_q, q);
      }
      odom_base.updateJointPositions(last_q);
      for (int i = 0; i < 4; ++i) {
        odom_base.legs[i]->in_contact(odom_base.legs[i]->gait_phase());
      }
      if (t % (ctrl_hz / odom_hz) == 0) {
        champ::Velocities est;
        odom.getVelocities(est, now_us);
        sum_wz += est.angular.z;
        ++odom_samples;
      }
    }
    const float mean_wz = static_cast<float>(sum_wz / std::max(1, odom_samples));
    check(mean_wz > 0.3f * cmd_wz && mean_wz < 1.8f * cmd_wz,
      "odom yaw rate same order as commanded wz",
      fmt("mean wz=%.3f (cmd %.3f); /2 patch should prevent ~2x", mean_wz, cmd_wz));
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
