// Offline gait continuity regression against actual CHAMP headers + robot files.
// No ROS, no MuJoCo: URDF/YAML kinematics only (the positions CHAMP commands).
//
//   measure_gait_continuity <urdf> <gait.yaml> <joints.yaml> <links.yaml>
//
// Catches the discontinuities that look like "jerky / only-backward" walking:
// signed vy+r*wz cancellation resetting the phase clock, Bezier endpoint
// velocity mismatch at stance↔swing, first-swing pops, and a hard zero
// cmd_vel slamming a swinging foot. The node stop path is the ROS-free
// velocity_slew.h used by quadruped_controller_node.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <quadruped_base/quadruped_base.h>

#include "champ_robot_config.h"
#include "velocity_slew.h"

namespace
{
constexpr double kDt = 0.005;
constexpr champ::PhaseGenerator::Time kDtUs = 5000ul;

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

bool joints_finite(const float joints[12])
{
  for (int i = 0; i < 12; ++i) {
    if (!std::isfinite(joints[i])) {
      return false;
    }
  }
  return true;
}

struct TickSample
{
  float x[4]{};
  float y[4]{};
  float z[4]{};
  bool stance[4]{};
};

struct WalkSim
{
  champ::GaitConfig gait;
  champ::QuadrupedBase base;
  champ::BodyController body;
  champ::LegController legs;
  champ::Kinematics kin;
  champ::Pose pose;
  champ_gait::PlanarVel current;
  bool stance[4]{true, true, true, true};
  bool touchdown{false};

  explicit WalkSim(const champ_tools::RobotConfig & cfg)
  : gait(cfg.champGait()),
    base(gait),
    body(base),
    legs(base, 0),
    kin(base)
  {
    cfg.applyTo(base);
    pose.position.z = gait.nominal_height;
  }

  TickSample tick(
    champ_gait::PlanarVel target, champ::PhaseGenerator::Time now_us,
    double acc_lin, double acc_ang, bool use_slew)
  {
    if (use_slew) {
      const bool allow_zero = touchdown || champ_gait::allStance(stance);
      current = champ_gait::slewPlanarVel(
        current, target, acc_lin, acc_ang, kDt, allow_zero);
    } else {
      current = target;
    }
    geometry::Transformation feet[4];
    body.poseCommand(feet, pose);
    champ::Velocities cmd;
    cmd.linear.x = static_cast<float>(current.vx);
    cmd.linear.y = static_cast<float>(current.vy);
    cmd.angular.z = static_cast<float>(current.wz);
    legs.velocityCommand(feet, cmd, now_us);
    touchdown = false;
    TickSample sample;
    for (int i = 0; i < 4; ++i) {
      const bool st = base.legs[i]->gait_phase();
      touchdown = touchdown || (st && !stance[i]);
      stance[i] = st;
      sample.x[i] = feet[i].X();
      sample.y[i] = feet[i].Y();
      sample.z[i] = feet[i].Z();
      sample.stance[i] = st;
    }
    return sample;
  }

  bool inverseFinite(const TickSample & sample)
  {
    geometry::Transformation feet[4];
    for (int i = 0; i < 4; ++i) {
      feet[i].X() = sample.x[i];
      feet[i].Y() = sample.y[i];
      feet[i].Z() = sample.z[i];
    }
    float q[12];
    for (int i = 0; i < 12; ++i) {
      q[i] = std::numeric_limits<float>::quiet_NaN();
    }
    kin.inverse(q, feet);
    return joints_finite(q);
  }
};

float maxAbsDz(const TickSample & a, const TickSample & b)
{
  float m = 0.0f;
  for (int i = 0; i < 4; ++i) {
    m = std::max(m, std::fabs(b.z[i] - a.z[i]));
  }
  return m;
}

float maxD3(const TickSample & a, const TickSample & b)
{
  float m = 0.0f;
  for (int i = 0; i < 4; ++i) {
    const float dx = b.x[i] - a.x[i];
    const float dy = b.y[i] - a.y[i];
    const float dz = b.z[i] - a.z[i];
    m = std::max(m, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  return m;
}

champ_gait::PlanarVel cancellationTwist(champ::QuadrupedBase & base, const champ::GaitConfig & gait)
{
  const float radius = std::max(1e-4f, base.lf.center_to_nominal());
  float vy = -std::min(0.10f, 0.5f * gait.max_linear_velocity_y);
  float wz = -vy / radius;  // vy + radius*wz == 0 (the old signed-sum zero)
  const float max_wz = gait.max_angular_velocity_z;
  if (std::fabs(wz) > max_wz && max_wz > 0.0f) {
    const float scale = max_wz / std::fabs(wz);
    wz *= scale;
    vy *= scale;
  }
  champ_gait::PlanarVel t;
  t.vy = vy;
  t.wz = wz;
  return t;
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

  const champ::GaitConfig gait = cfg.champGait();
  const double acc_lin = cfg.gait.max_linear_acceleration;
  const double acc_ang = cfg.gait.max_angular_acceleration;
  const float dt = static_cast<float>(kDt);
  std::printf(
    "Robot '%s' continuity: stance %.3f s swing %.3f s max_vx %.3f\n",
    cfg.urdf.name.c_str(), gait.stance_duration, gait.swing_duration,
    gait.max_linear_velocity_x);

  {
    champ_gait::PlanarVel cur{0.2, 0.0, 0.0};
    champ_gait::PlanarVel tgt{0.0, 0.0, 0.0};
    const auto blocked = champ_gait::slewPlanarVel(cur, tgt, 1.0, 1.0, 1.0, false);
    check(
      !blocked.isZero() && blocked.vx == 0.2,
      "velocity_slew holds last vel when a hard zero is not allowed");
    const auto allowed = champ_gait::slewPlanarVel(cur, tgt, 1.0, 1.0, 1.0, true);
    check(allowed.isZero(), "velocity_slew allows zero at touchdown / all-stance");
  }

  geometry::Transformation stand_feet[4];
  {
    WalkSim sim(cfg);
    sim.body.poseCommand(stand_feet, sim.pose);
    std::printf("LF center_to_nominal %.6f m\n", sim.base.lf.center_to_nominal());
  }

  // --- First swing leaves the ground; +vx stance feet push -X; C1 endpoints ---
  {
    WalkSim sim(cfg);
    champ_gait::PlanarVel target;
    target.vx = gait.max_linear_velocity_x;
    const int ticks = 400;
    std::vector<TickSample> hist;
    hist.reserve(static_cast<size_t>(ticks));
    float first_lift = -1.0f;
    int ik_fail = 0;
    for (int t = 0; t < ticks; ++t) {
      const TickSample s = sim.tick(
        target, static_cast<champ::PhaseGenerator::Time>(t) * kDtUs, 0.0, 0.0, false);
      hist.push_back(s);
      if (!sim.inverseFinite(s)) {
        ++ik_fail;
      }
      if (first_lift < 0.0f) {
        for (int i = 0; i < 4; ++i) {
          if (!s.stance[i]) {
            first_lift = std::max(0.0f, s.z[i] - stand_feet[i].Z());
            break;
          }
        }
      }
    }

    check(ik_fail == 0, "forward walk IK stays finite", fmt("fail=%d/%d", ik_fail, ticks));
    check(
      first_lift >= 0.0f && first_lift < 0.005f,
      "first swing leaves the ground (not a full swing_height pop)",
      fmt("first lift %.4f mm, swing_height %.1f mm", first_lift * 1000.0f,
        gait.swing_height * 1000.0f));

    double sum_stance_vx = 0.0;
    int n_stance_vx = 0;
    double sum_swing_start = 0.0;
    double sum_swing_end = 0.0;
    int n_swing_start = 0;
    int n_swing_end = 0;
    float mean_start_vx = 0.0f;
    float mean_end_vx = 0.0f;
    const int skip = 80;
    int swing_age = 0;
    for (int t = 0; t < skip; ++t) {
      swing_age = hist[static_cast<size_t>(t)].stance[0] ? 0 : swing_age + 1;
    }
    for (int t = skip; t < ticks; ++t) {
      const float vx = (hist[static_cast<size_t>(t)].x[0] -
        hist[static_cast<size_t>(t - 1)].x[0]) / dt;
      const bool st0 = hist[static_cast<size_t>(t)].stance[0];
      const bool st1 = hist[static_cast<size_t>(t - 1)].stance[0];
      if (st1 && st0) {
        sum_stance_vx += vx;
        ++n_stance_vx;
      }
      if (!st0) {
        ++swing_age;
        // First interior swing interval (tick 1 -> 2), closest 5 ms sample
        // to the Bezier start derivative n*(P1-P0)/Tw.
        if (swing_age == 2) {
          sum_swing_start += vx;
          ++n_swing_start;
        }
      } else {
        if (swing_age >= 3) {
          const float v_end = (hist[static_cast<size_t>(t - 2)].x[0] -
            hist[static_cast<size_t>(t - 3)].x[0]) / dt;
          sum_swing_end += v_end;
          ++n_swing_end;
        }
        swing_age = 0;
      }
    }
    const float mean_stance_vx = n_stance_vx > 0 ?
      static_cast<float>(sum_stance_vx / n_stance_vx) : 0.0f;
    mean_start_vx = n_swing_start > 0 ?
      static_cast<float>(sum_swing_start / n_swing_start) : 0.0f;
    mean_end_vx = n_swing_end > 0 ?
      static_cast<float>(sum_swing_end / n_swing_end) : 0.0f;

    check(
      n_stance_vx > 10 && mean_stance_vx < -0.4f * gait.max_linear_velocity_x,
      "+vx stance feet push -X (body forward; this is the cmd_vel sign)",
      fmt("mean stance vx %.3f m/s, cmd +%.3f", mean_stance_vx, gait.max_linear_velocity_x));
    check(
      n_swing_start > 0 && std::fabs(mean_start_vx - mean_stance_vx) < 0.22f &&
      std::fabs(mean_start_vx) < 1.2f,
      "swing start X velocity matches stance (C1)",
      fmt("stance %.3f, swing start %.3f m/s, n=%d (pre-fix ~1.6 m/s)",
        mean_stance_vx, mean_start_vx, n_swing_start));
    check(
      n_swing_end > 0 && std::fabs(mean_end_vx - mean_stance_vx) < 0.22f &&
      std::fabs(mean_end_vx) < 1.2f,
      "swing end X velocity matches stance (C1)",
      fmt("stance %.3f, swing end %.3f m/s, n=%d",
        mean_stance_vx, mean_end_vx, n_swing_end));
  }

  // --- Signed cancellation: vy + r*wz == 0 must not reset the phase clock ---
  {
    WalkSim sim(cfg);
    champ_gait::PlanarVel forward;
    forward.vx = 0.8f * gait.max_linear_velocity_x;
    const champ_gait::PlanarVel cancel = cancellationTwist(sim.base, gait);
    const float signed_sum = static_cast<float>(cancel.vy) +
      sim.base.lf.center_to_nominal() * static_cast<float>(cancel.wz);
    check(
      std::fabs(signed_sum) < 1e-4f && (std::fabs(cancel.vy) > 1e-4 || std::fabs(cancel.wz) > 1e-4),
      "cancellation case is a nonzero twist whose signed (vy + r wz) is ~0",
      fmt("vy=%.4f wz=%.4f vy+r*wz=%.6f r=%.6f", cancel.vy, cancel.wz, signed_sum,
        sim.base.lf.center_to_nominal()));

    TickSample prev{};
    float peak_swing = 0.0f;
    int swing_leg = -1;
    const int warmup = 120;
    for (int t = 0; t < warmup; ++t) {
      const TickSample s = sim.tick(
        forward, static_cast<champ::PhaseGenerator::Time>(t) * kDtUs, 0.0, 0.0, false);
      for (int i = 0; i < 4; ++i) {
        if (!s.stance[i]) {
          const float lift = s.z[i] - stand_feet[i].Z();
          if (lift > peak_swing) {
            peak_swing = lift;
            swing_leg = i;
          }
        }
      }
      prev = s;
    }
    check(
      peak_swing > 0.4f * gait.swing_height && swing_leg >= 0,
      "warmup reached a mid-swing foot before cancellation",
      fmt("peak lift %.1f mm", peak_swing * 1000.0f));

    float max_dz = 0.0f;
    bool stayed = true;
    const int hold = 8;
    for (int t = 0; t < hold; ++t) {
      const TickSample s = sim.tick(
        cancel, static_cast<champ::PhaseGenerator::Time>(warmup + t) * kDtUs, 0.0, 0.0, false);
      stayed = stayed && sim.legs.phase_generator.has_started;
      max_dz = std::max(max_dz, maxAbsDz(prev, s));
      prev = s;
    }
    for (int t = 0; t < 4; ++t) {
      const TickSample s = sim.tick(
        forward,
        static_cast<champ::PhaseGenerator::Time>(warmup + hold + t) * kDtUs, 0.0, 0.0, false);
      stayed = stayed && sim.legs.phase_generator.has_started;
      max_dz = std::max(max_dz, maxAbsDz(prev, s));
      prev = s;
    }
    check(
      stayed,
      "cancellation twist does not reset the phase clock",
      "old signed norm treated this as a stop and dropped the swinging foot");
    check(
      max_dz < 0.020f,
      "no mid-swing Z drop across cancellation + resume",
      fmt("max dZ %.3f mm (was ~95 mm with the signed-sum bug)", max_dz * 1000.0f));
  }

  // --- Stop through the node slew gate at every phase offset of one stride ---
  {
    std::printf("slew accelerations %.3f m/s^2, %.3f rad/s^2\n", acc_lin, acc_ang);
    const float stride = gait.stance_duration + gait.swing_duration;
    const int stride_ticks = std::max(1, static_cast<int>(std::lround(stride / kDt)));
    champ_gait::PlanarVel walk;
    walk.vx = 0.5f * gait.max_linear_velocity_x;
    champ_gait::PlanarVel stop;
    float worst_z = 0.0f;
    float worst_3d = 0.0f;
    int worst_offset = 0;
    int failed_offsets = 0;
    int stopped = 0;
    for (int offset = 0; offset < stride_ticks; ++offset) {
      WalkSim sim(cfg);
      TickSample prev{};
      bool have = false;
      float zero_z = 0.0f;
      float zero_3d = 0.0f;
      bool saw_zero = false;
      const int warmup = 2 * stride_ticks + offset;
      for (int t = 0; t < warmup + 2000; ++t) {
        const champ_gait::PlanarVel target = t < warmup ? walk : stop;
        const TickSample s = sim.tick(
          target, static_cast<champ::PhaseGenerator::Time>(t) * kDtUs,
          acc_lin, acc_ang, true);
        if (t >= warmup && have && !saw_zero && sim.current.isZero()) {
          zero_z = maxAbsDz(prev, s);
          zero_3d = maxD3(prev, s);
          saw_zero = true;
          break;
        }
        prev = s;
        have = true;
      }
      if (!saw_zero) {
        ++failed_offsets;
        continue;
      }
      ++stopped;
      if (zero_z > 0.008f || zero_3d > 0.020f) {
        ++failed_offsets;
      }
      if (zero_z > worst_z || zero_3d > worst_3d) {
        worst_z = std::max(worst_z, zero_z);
        worst_3d = std::max(worst_3d, zero_3d);
        worst_offset = offset;
      }
    }
    check(
      failed_offsets == 0 && stopped == stride_ticks,
      fmt("node stop gate: %d phase offsets, no swing slam at the zeroing tick", stride_ticks),
      fmt("worst dZ %.3f mm 3D %.3f mm at offset %d (stopped %d)",
        worst_z * 1000.0f, worst_3d * 1000.0f, worst_offset, stopped));
  }

  // --- Core contract: a hard zero mid-swing still plants (callers must not skip the gate) ---
  {
    WalkSim sim(cfg);
    champ_gait::PlanarVel walk;
    walk.vx = gait.max_linear_velocity_x;
    TickSample prev{};
    bool have = false;
    float peak = 0.0f;
    int t_hit = -1;
    for (int t = 0; t < 200; ++t) {
      const TickSample s = sim.tick(
        walk, static_cast<champ::PhaseGenerator::Time>(t) * kDtUs, 0.0, 0.0, false);
      for (int i = 0; i < 4; ++i) {
        if (!s.stance[i]) {
          peak = std::max(peak, s.z[i] - stand_feet[i].Z());
        }
      }
      if (peak > 0.6f * gait.swing_height) {
        t_hit = t;
        prev = s;
        have = true;
        break;
      }
      prev = s;
      have = true;
    }
    float slam = 0.0f;
    if (have && t_hit >= 0) {
      champ_gait::PlanarVel zero;
      const TickSample s = sim.tick(
        zero, static_cast<champ::PhaseGenerator::Time>(t_hit + 1) * kDtUs, 0.0, 0.0, false);
      slam = maxAbsDz(prev, s);
    }
    check(
      t_hit >= 0 && slam > 0.5f * peak,
      "LegController still slams a swinging foot on a hard zero (do not bypass the node gate)",
      fmt("dZ %.1f mm from %.1f mm swing", slam * 1000.0f, peak * 1000.0f));
  }

  std::printf("%d checks, %d failed\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
