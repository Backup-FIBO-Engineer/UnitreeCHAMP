// Dump a CHAMP joint trajectory for Go2 or B2 (no ROS). Used by
// verify_mujoco_walk.py.
//
//   dump_unitree_gait <go2|b2> [vx vy wz ticks swing stance]
//
// Prints one line per 5 ms tick: 12 joints in CHAMP order FL FR RL RR
// (hip, thigh, calf). With ticks=0 prints only the standing pose and the
// standing foot positions in the base frame (used to lock the MuJoCo checks).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <quadruped_base/quadruped_base.h>

namespace
{
struct LegXyz
{
  float hip[3];
  float upper[3];
  float lower[3];
  float foot[3];
};

struct RobotSpec
{
  const char * name;
  LegXyz legs[4];  // LF, RF, LH, RH
  float nominal_height;
  float swing_height;
  float stance_duration;
};

// urdf/go2.urdf joint origins (rpy=0).
const RobotSpec kGo2 = {
  "go2",
  {
    {{0.1934f, 0.0465f, 0.0f}, {0.0f, 0.0955f, 0.0f}, {0.0f, 0.0f, -0.213f}, {0.0f, 0.0f, -0.213f}},
    {{0.1934f, -0.0465f, 0.0f}, {0.0f, -0.0955f, 0.0f}, {0.0f, 0.0f, -0.213f}, {0.0f, 0.0f, -0.213f}},
    {{-0.1934f, 0.0465f, 0.0f}, {0.0f, 0.0955f, 0.0f}, {0.0f, 0.0f, -0.213f}, {0.0f, 0.0f, -0.213f}},
    {{-0.1934f, -0.0465f, 0.0f}, {0.0f, -0.0955f, 0.0f}, {0.0f, 0.0f, -0.213f}, {0.0f, 0.0f, -0.213f}},
  },
  0.30f, 0.08f, 0.25f,
};

// urdf/b2.urdf joint origins (rpy=0). The calf joints carry the official
// +-8.7e-5 m y offset; RR_foot_joint also has y=-8.6984e-05.
const RobotSpec kB2 = {
  "b2",
  {
    {{0.3285f, 0.072f, 0.0f}, {0.0f, 0.11973f, 0.0f}, {0.0f, -8.6984e-05f, -0.35f}, {0.0f, 0.0f, -0.35f}},
    {{0.3285f, -0.072f, 0.0f}, {0.0f, -0.11973f, 0.0f}, {0.0f, 8.6986e-05f, -0.35f}, {0.0f, 0.0f, -0.35f}},
    {{-0.3285f, 0.072f, 0.0f}, {0.0f, 0.11973f, 0.0f}, {0.0f, -8.6984e-05f, -0.35f}, {0.0f, 0.0f, -0.35f}},
    {{-0.3285f, -0.072f, 0.0f}, {0.0f, -0.11973f, 0.0f}, {0.0f, 8.6986e-05f, -0.35f}, {0.0f, -8.6984e-05f, -0.35f}},
  },
  0.50f, 0.10f, 0.30f,
};

void apply_leg(champ::QuadrupedLeg & leg, const LegXyz & xyz)
{
  leg.hip.setTranslation(xyz.hip[0], xyz.hip[1], xyz.hip[2]);
  leg.upper_leg.setTranslation(xyz.upper[0], xyz.upper[1], xyz.upper[2]);
  leg.lower_leg.setTranslation(xyz.lower[0], xyz.lower[1], xyz.lower[2]);
  leg.foot.setTranslation(xyz.foot[0], xyz.foot[1], xyz.foot[2]);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <go2|b2> [vx vy wz ticks swing stance]\n", argv[0]);
    return 2;
  }
  const RobotSpec * spec = nullptr;
  if (std::strcmp(argv[1], "go2") == 0) {
    spec = &kGo2;
  } else if (std::strcmp(argv[1], "b2") == 0) {
    spec = &kB2;
  } else {
    std::fprintf(stderr, "unknown robot '%s' (go2|b2)\n", argv[1]);
    return 2;
  }

  const float vx = (argc > 2) ? static_cast<float>(std::atof(argv[2])) : 0.25f;
  const float vy = (argc > 3) ? static_cast<float>(std::atof(argv[3])) : 0.0f;
  const float wz = (argc > 4) ? static_cast<float>(std::atof(argv[4])) : 0.0f;
  const int ticks = (argc > 5) ? std::atoi(argv[5]) : 800;
  const float swing = (argc > 6) ? static_cast<float>(std::atof(argv[6])) : spec->swing_height;
  const float stance = (argc > 7) ? static_cast<float>(std::atof(argv[7])) : spec->stance_duration;

  static const char kKnee[] = ">>";
  champ::GaitConfig gait;
  gait.knee_orientation = kKnee;
  gait.odom_scaler = 0.9f;
  gait.max_linear_velocity_x = 1.00f;
  gait.max_linear_velocity_y = 0.50f;
  gait.max_angular_velocity_z = 1.20f;
  gait.com_x_translation = 0.0f;
  gait.swing_height = swing;
  gait.stance_depth = 0.0f;
  gait.stance_duration = stance;
  gait.nominal_height = spec->nominal_height;

  champ::QuadrupedBase base(gait);
  apply_leg(base.lf, spec->legs[0]);
  apply_leg(base.rf, spec->legs[1]);
  apply_leg(base.lh, spec->legs[2]);
  apply_leg(base.rh, spec->legs[3]);

  champ::BodyController body(base);
  champ::LegController legs(base, 0);
  champ::Kinematics kin(base);
  champ::Pose pose;
  pose.position.z = gait.nominal_height;

  if (ticks <= 0) {
    geometry::Transformation feet[4];
    body.poseCommand(feet, pose);
    float q[12] = {};
    kin.inverse(q, feet);
    base.updateJointPositions(q);
    std::printf("stand_joints");
    for (int i = 0; i < 12; ++i) {
      std::printf(" %.9g", q[i]);
    }
    std::printf("\n");
    for (int i = 0; i < 4; ++i) {
      geometry::Transformation fb = base.legs[i]->foot_from_base();
      std::printf("stand_foot_%d %.10g %.10g %.10g\n", i, fb.X(), fb.Y(), fb.Z());
    }
    return 0;
  }

  float last[12] = {};
  for (int t = 0; t < ticks; ++t) {
    geometry::Transformation feet[4];
    body.poseCommand(feet, pose);
    champ::Velocities cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    legs.velocityCommand(
      feet, cmd, static_cast<champ::PhaseGenerator::Time>(t) * 5000ul);
    float q[12];
    for (int i = 0; i < 12; ++i) {
      q[i] = last[i];
    }
    kin.inverse(q, feet);
    for (int i = 0; i < 12; ++i) {
      if (!std::isfinite(q[i])) {
        q[i] = last[i];
      }
      last[i] = q[i];
      std::printf("%s%.9g", i ? " " : "", q[i]);
    }
    std::printf("\n");
  }
  return 0;
}
