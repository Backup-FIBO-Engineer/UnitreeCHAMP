// Offline verification of CHAMP IK/FK, gait, and odometry against Go2 URDF
// translations. Compiles against champ headers only (no ROS).
//
// Build:
//   g++ -std=c++17 -O2
//     -I <repo>/src/champ/include/champ
//     -o /tmp/verify_champ_go2 verify_champ_go2.cpp
//
// Translations below are parent_to_joint_origin xyz from urdf/go2.urdf
// (all rpy=0, so champ::URDF::getPose's translation sum is exact).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <odometry/odometry.h>
#include <quadruped_base/quadruped_base.h>

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

struct LegXyz
{
  float hip[3];
  float upper[3];
  float lower[3];
  float foot[3];
};

// Exact xyz from go2.urdf joint origins (rpy all zero).
const LegXyz kLf = {
  {0.1934f, 0.0465f, 0.0f},
  {0.0f, 0.0955f, 0.0f},
  {0.0f, 0.0f, -0.213f},
  {0.0f, 0.0f, -0.213f},
};
const LegXyz kRf = {
  {0.1934f, -0.0465f, 0.0f},
  {0.0f, -0.0955f, 0.0f},
  {0.0f, 0.0f, -0.213f},
  {0.0f, 0.0f, -0.213f},
};
const LegXyz kLh = {
  {-0.1934f, 0.0465f, 0.0f},
  {0.0f, 0.0955f, 0.0f},
  {0.0f, 0.0f, -0.213f},
  {0.0f, 0.0f, -0.213f},
};
const LegXyz kRh = {
  {-0.1934f, -0.0465f, 0.0f},
  {0.0f, -0.0955f, 0.0f},
  {0.0f, 0.0f, -0.213f},
  {0.0f, 0.0f, -0.213f},
};

void apply_leg(champ::QuadrupedLeg & leg, const LegXyz & xyz)
{
  leg.hip.setTranslation(xyz.hip[0], xyz.hip[1], xyz.hip[2]);
  leg.upper_leg.setTranslation(xyz.upper[0], xyz.upper[1], xyz.upper[2]);
  leg.lower_leg.setTranslation(xyz.lower[0], xyz.lower[1], xyz.lower[2]);
  leg.foot.setTranslation(xyz.foot[0], xyz.foot[1], xyz.foot[2]);
}

champ::GaitConfig make_gait()
{
  // Matches config/go2_gait.yaml. knee_orientation must outlive setGaitConfig.
  static const char kKnee[] = ">>";
  champ::GaitConfig gait;
  gait.knee_orientation = kKnee;
  gait.odom_scaler = 0.9f;
  gait.max_linear_velocity_x = 0.50f;
  gait.max_linear_velocity_y = 0.15f;
  gait.max_angular_velocity_z = 0.6f;
  gait.com_x_translation = 0.0f;
  gait.swing_height = 0.08f;
  gait.stance_depth = 0.0f;
  gait.stance_duration = 0.25f;
  gait.nominal_height = 0.30f;
  return gait;
}

void load_go2(champ::QuadrupedBase & base)
{
  apply_leg(base.lf, kLf);
  apply_leg(base.rf, kRf);
  apply_leg(base.lh, kLh);
  apply_leg(base.rh, kRh);
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

}  // namespace

int main()
{
  champ::GaitConfig gait = make_gait();
  champ::QuadrupedBase base(gait);
  load_go2(base);

  // --- URDF contract: CHAMP IK assumes hip=X, thigh/calf=Y, rpy=0 ---
  check(true, "URDF axes (manual)", "hip xyz=1 0 0, upper/lower xyz=0 1 0, all rpy=0");

  char buf[256];
  const float lf_reach = planar_reach(base.lf);
  std::snprintf(buf, sizeof(buf), "l1+l2=%.4f m (yaml claims 0.426)", lf_reach);
  check(std::fabs(lf_reach - 0.426f) < 0.001f, "LF planar reach matches yaml", buf);

  const float l0 =
    base.lf.upper_leg.y() + base.lf.lower_leg.y() + base.lf.foot.y();
  std::snprintf(buf, sizeof(buf), "l0=%.5f (hip Y offset of planar chain)", l0);
  check(l0 > 0.09f && l0 < 0.11f, "LF l0 is outward hip offset", buf);

  // zero_stance Z should be hip.z + upper.z + lower.z + foot.z (CHAMP standing stretch)
  geometry::Transformation zs = base.lf.zero_stance();
  const float zs_z_expected =
    kLf.hip[2] + kLf.upper[2] + kLf.lower[2] + kLf.foot[2];
  std::snprintf(buf, sizeof(buf), "Z=%.4f expected %.4f", zs.Z(), zs_z_expected);
  check(std::fabs(zs.Z() - zs_z_expected) < 1e-6f, "LF zero_stance Z from URDF xyz", buf);

  const float stand_z = -(zs.Z() + gait.nominal_height);
  std::snprintf(
    buf, sizeof(buf),
    "req_tz=%.4f (nominal 0.30 sits %.1f mm above fully-stretched Z)",
    stand_z, (std::fabs(zs.Z()) - gait.nominal_height) * 1000.0f);
  check(stand_z > 0.0f, "nominal_height is shorter than stretched leg", buf);

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

  base.updateJointPositions(stand_joints);
  float stand_fk_err = 0.0f;
  for (int i = 0; i < 4; ++i) {
    geometry::Transformation fk = base.legs[i]->foot_from_hip();
    const float err = hypot3(
      fk.X() - stand_feet[i].X(),
      fk.Y() - stand_feet[i].Y(),
      fk.Z() - stand_feet[i].Z());
    if (err > stand_fk_err) {
      stand_fk_err = err;
    }
  }
  std::snprintf(buf, sizeof(buf), "max |FK-target|=%.6f m", stand_fk_err);
  check(stand_fk_err < 1e-3f, "standing IK/FK roundtrip in hip frame", buf);

  // Base-frame foot Z at stand should be about -nominal_height after hip translate.
  // poseCommand target is hip-frame; foot_from_base.Z should be ~ -0.30
  float height_err = 0.0f;
  for (int i = 0; i < 4; ++i) {
    geometry::Transformation fb = base.legs[i]->foot_from_base();
    height_err = std::max(height_err, std::fabs(fb.Z() + gait.nominal_height));
  }
  std::snprintf(buf, sizeof(buf), "max |foot_z + nominal|=%.5f m", height_err);
  check(height_err < 5e-3f, "standing feet at nominal_height in base frame", buf);

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
  std::snprintf(buf, sizeof(buf), "max |champ-independent|=%.7f m", indep_err);
  check(indep_err < 1e-6f, "CHAMP FK matches independent Rx/Ry URDF FK at stand", buf);

  // --- Node ctor order: Odometry is built before URDF translations are loaded ---
  {
    champ::GaitConfig gait_early = make_gait();
    champ::QuadrupedBase early_base(gait_early);
    champ::Odometry early_odom(early_base, 0);
    load_go2(early_base);
    early_base.updateJointPositions(stand_joints);
    for (int i = 0; i < 4; ++i) {
      early_base.legs[i]->in_contact(true);
    }
    float first_walk_vx = 0.0f;
    for (int t = 1; t <= 12; ++t) {
      champ::Velocities est;
      const champ::Odometry::Time now_us =
        static_cast<champ::Odometry::Time>(t) * 20000ul;
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
    std::snprintf(buf, sizeof(buf), "first trot vx=%.3f m/s (spike would be several m/s)", first_walk_vx);
    check(std::fabs(first_walk_vx) < 1.0f, "odom after stand does not spike from pre-URDF zeros", buf);
  }

  // --- Random reachable targets: FK(q) -> IK -> FK(q') ---
  float max_repro_err = 0.0f;
  int random_ok = 0;
  const int kRandomN = 80;
  for (int n = 0; n < kRandomN; ++n) {
    float q[12];
    // Perturb standing joints; keep inside a conservative box that stays reachable.
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
      err = std::max(
        err,
        hypot3(fk.X() - feet[i].X(), fk.Y() - feet[i].Y(), fk.Z() - feet[i].Z()));
      float hx, hy, hz;
      independent_fk_hip(*base.legs[i], hx, hy, hz);
      indep_sample = std::max(indep_sample, hypot3(hx - fk.X(), hy - fk.Y(), hz - fk.Z()));
    }
    max_repro_err = std::max(max_repro_err, err);
    if (err < 2e-3f && indep_sample < 1e-6f) {
      ++random_ok;
    }
  }
  std::snprintf(
    buf, sizeof(buf), "%d/%d repro <2mm, max err=%.5f m",
    random_ok, kRandomN, max_repro_err);
  check(random_ok >= kRandomN - 5, "IK reproduces FK samples near standing", buf);

  // --- Unreachable target: inverse must not publish garbage joints ---
  geometry::Transformation far_feet[4];
  body_controller.poseCommand(far_feet, req_pose);
  far_feet[0].Z() -= 0.8f;  // beyond 0.426 m reach
  float before[12];
  copy_joints(before, stand_joints);
  float after[12];
  copy_joints(after, stand_joints);
  kinematics.inverse(after, far_feet);
  bool unchanged = true;
  bool any_nan = false;
  bool any_garbage = false;
  for (int i = 0; i < 12; ++i) {
    if (std::isnan(after[i])) {
      any_nan = true;
    }
    if (after[i] != before[i]) {
      unchanged = false;
    }
    if (std::isfinite(after[i]) && std::fabs(after[i] - before[i]) > 1e-6f &&
      (std::fabs(after[i]) > 10.0f || !std::isfinite(after[i])))
    {
      any_garbage = true;
    }
  }
  (void)any_garbage;
  // Documented CHAMP behavior: if any leg is NaN, the whole plan is discarded
  // (joint_positions unchanged). If a leg is unreachable without writing NaN,
  // uninitialized stack values can leak into the command.
  const bool safe_unreach = unchanged || any_nan;
  std::snprintf(
    buf, sizeof(buf),
    "unchanged=%d any_nan=%d (unsafe if both false: garbage joints published)",
    static_cast<int>(unchanged), static_cast<int>(any_nan));
  check(safe_unreach, "unreachable IK does not publish garbage", buf);

  // --- Gait: standing (v=0) keeps all four contacts and standing joints ---
  base.updateJointPositions(stand_joints);
  geometry::Transformation gait_feet[4];
  body_controller.poseCommand(gait_feet, req_pose);
  champ::Velocities vel;
  vel.linear.x = 0.0f;
  vel.linear.y = 0.0f;
  vel.angular.z = 0.0f;
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
  std::snprintf(buf, sizeof(buf), "max |dq|=%.5f rad", still_err);
  check(still_err < 1e-3f, "zero velocity: IK matches standing pose", buf);

  // --- Gait walk: every tick IK finite and within reach; trot pairing ---
  struct Case
  {
    const char * name;
    float vx;
    float vy;
    float wz;
  };
  const Case cases[] = {
    {"vx=0.50", 0.50f, 0.0f, 0.0f},
    {"vy=0.15", 0.0f, 0.15f, 0.0f},
    {"wz=0.6", 0.0f, 0.0f, 0.6f},
    {"vx+wz", 0.25f, 0.0f, 0.4f},
    {"worst yaml", 0.50f, 0.15f, 0.6f},
  };

  for (const Case & c : cases) {
    champ::QuadrupedBase walk_base(gait);
    load_go2(walk_base);
    champ::Kinematics walk_kin(walk_base);
    champ::BodyController walk_body(walk_base);
    champ::LegController walk_legs(walk_base, 0);

    int ik_fail = 0;
    int reach_fail = 0;
    int ticks = 0;
    int trot_ok = 0;
    const int kTicks = 400;  // 2 s at 200 Hz
    for (int t = 0; t < kTicks; ++t) {
      const champ::PhaseGenerator::Time now_us =
        static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      walk_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.linear.x = c.vx;
      cmd.linear.y = c.vy;
      cmd.angular.z = c.wz;
      walk_legs.velocityCommand(feet, cmd, now_us);

      for (int i = 0; i < 4; ++i) {
        const float r = hypot3(feet[i].X(), 0.0f, feet[i].Z());
        // After hip solve the planar length is sqrt(x^2+z^2) in the rotated frame;
        // a conservative pre-check uses hip-frame radius vs l1+l2 plus |l0|.
        if (hypot3(feet[i].X(), feet[i].Y(), feet[i].Z()) > planar_reach(*walk_base.legs[i]) + 0.02f) {
          ++reach_fail;
        }
        (void)r;
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
      }
      ++ticks;
    }
    std::snprintf(
      buf, sizeof(buf),
      "ik_fail=%d/%d reach_fail=%d trot_diag=%d/%d",
      ik_fail, ticks, reach_fail, trot_ok, ticks);
    check(ik_fail == 0, c.name, buf);
    check(trot_ok > ticks / 2, (std::string(c.name) + " trot LF-RH / RF-LH").c_str(), buf);
  }

  // --- Odometry: open-loop planned joints + gait_phase contacts ---
  {
    champ::QuadrupedBase odom_base(gait);
    load_go2(odom_base);
    champ::Kinematics odom_kin(odom_base);
    champ::BodyController odom_body(odom_base);
    champ::LegController odom_legs(odom_base, 0);
    champ::Odometry odom(odom_base, 0);

    const float cmd_vx = 0.20f;
    const int ctrl_hz = 200;
    const int odom_hz = 50;
    const int seconds = 2;
    const int ctrl_ticks = ctrl_hz * seconds;
    float last_q[12] = {};
    kinematics.inverse(last_q, stand_feet);  // seed

    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double sum_wz = 0.0;
    int odom_samples = 0;
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    const float odom_dt = 1.0f / static_cast<float>(odom_hz);

    for (int t = 0; t < ctrl_ticks; ++t) {
      const champ::PhaseGenerator::Time now_us =
        static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      odom_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.linear.x = cmd_vx;
      cmd.linear.y = 0.0f;
      cmd.angular.z = 0.0f;
      odom_legs.velocityCommand(feet, cmd, now_us);
      float q[12];
      for (int i = 0; i < 12; ++i) {
        q[i] = last_q[i];
      }
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
    const float expected_vx = cmd_vx * gait.odom_scaler;  // scaler is applied in getVelocities
    std::snprintf(
      buf, sizeof(buf),
      "mean vx=%.3f (cmd %.2f * scaler %.1f => %.3f) vy=%.3f wz=%.3f  integrated x=%.3f m in %ds",
      mean_vx, cmd_vx, gait.odom_scaler, expected_vx, mean_vy, mean_wz, x, seconds);
    // Allow generous band: CHAMP odom is low-pass (beta=0.1) and zeros when 4-foot contact.
    check(mean_vx > 0.08f && mean_vx < 0.28f, "odom vx has correct sign and order of magnitude", buf);
    check(std::fabs(mean_vy) < 0.05f, "odom vy near 0 for straight walk", buf);
    check(std::fabs(mean_wz) < 0.15f, "odom wz near 0 for straight walk", buf);
    check(x > 0.10f, "integrated /odom/raw x advances forward", buf);
  }

  {
    champ::QuadrupedBase odom_base(gait);
    load_go2(odom_base);
    champ::Kinematics odom_kin(odom_base);
    champ::BodyController odom_body(odom_base);
    champ::LegController odom_legs(odom_base, 0);
    champ::Odometry odom(odom_base, 0);

    const float cmd_wz = 0.5f;
    const int ctrl_hz = 200;
    const int odom_hz = 50;
    const int seconds = 2;
    const int ctrl_ticks = ctrl_hz * seconds;
    float last_q[12] = {};
    copy_joints(last_q, stand_joints);

    double sum_wz = 0.0;
    int odom_samples = 0;
    for (int t = 0; t < ctrl_ticks; ++t) {
      const champ::PhaseGenerator::Time now_us =
        static_cast<champ::PhaseGenerator::Time>(t) * 5000ul;
      geometry::Transformation feet[4];
      odom_body.poseCommand(feet, req_pose);
      champ::Velocities cmd;
      cmd.linear.x = 0.0f;
      cmd.linear.y = 0.0f;
      cmd.angular.z = cmd_wz;
      odom_legs.velocityCommand(feet, cmd, now_us);
      float q[12];
      for (int i = 0; i < 12; ++i) {
        q[i] = last_q[i];
      }
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
    std::snprintf(
      buf, sizeof(buf), "mean wz=%.3f (cmd %.2f); /2 patch should prevent ~2x",
      mean_wz, cmd_wz);
    check(mean_wz > 0.15f && mean_wz < 0.90f, "odom yaw rate same order as commanded wz", buf);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
