// Dump CHAMP Go2 joint trajectory (no ROS). Used by verify_mujoco_walk.py.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <quadruped_base/quadruped_base.h>

namespace
{
void apply_leg(champ::QuadrupedLeg & leg, float hx, float hy, float hz,
  float ux, float uy, float uz, float lx, float ly, float lz,
  float fx, float fy, float fz)
{
  leg.hip.setTranslation(hx, hy, hz);
  leg.upper_leg.setTranslation(ux, uy, uz);
  leg.lower_leg.setTranslation(lx, ly, lz);
  leg.foot.setTranslation(fx, fy, fz);
}
}  // namespace

int main(int argc, char ** argv)
{
  const float vx = (argc > 1) ? static_cast<float>(std::atof(argv[1])) : 0.25f;
  const float vy = (argc > 2) ? static_cast<float>(std::atof(argv[2])) : 0.0f;
  const float wz = (argc > 3) ? static_cast<float>(std::atof(argv[3])) : 0.0f;
  const int ticks = (argc > 4) ? std::atoi(argv[4]) : 800;
  const float swing = (argc > 5) ? static_cast<float>(std::atof(argv[5])) : 0.08f;
  const float stance = (argc > 6) ? static_cast<float>(std::atof(argv[6])) : 0.25f;

  static const char kKnee[] = ">>";
  champ::GaitConfig gait;
  gait.knee_orientation = kKnee;
  gait.odom_scaler = 0.9f;
  gait.max_linear_velocity_x = 0.80f;
  gait.max_linear_velocity_y = 0.40f;
  gait.max_angular_velocity_z = 1.20f;
  gait.com_x_translation = 0.0f;
  gait.swing_height = swing;
  gait.stance_depth = 0.0f;
  gait.stance_duration = stance;
  gait.nominal_height = 0.30f;

  champ::QuadrupedBase base(gait);
  apply_leg(base.lf, 0.1934f, 0.0465f, 0, 0, 0.0955f, 0, 0, 0, -0.213f, 0, 0, -0.213f);
  apply_leg(base.rf, 0.1934f, -0.0465f, 0, 0, -0.0955f, 0, 0, 0, -0.213f, 0, 0, -0.213f);
  apply_leg(base.lh, -0.1934f, 0.0465f, 0, 0, 0.0955f, 0, 0, 0, -0.213f, 0, 0, -0.213f);
  apply_leg(base.rh, -0.1934f, -0.0465f, 0, 0, -0.0955f, 0, 0, 0, -0.213f, 0, 0, -0.213f);

  champ::BodyController body(base);
  champ::LegController legs(base, 0);
  champ::Kinematics kin(base);
  champ::Pose pose;
  pose.position.z = gait.nominal_height;

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
