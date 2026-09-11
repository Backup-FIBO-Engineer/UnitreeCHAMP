// Dump a CHAMP joint trajectory for any robot described by URDF + yaml (no ROS).
// Used by verify_mujoco_walk.py and verify_mujoco_physics.py.
//
//   dump_champ_gait <urdf> <gait.yaml> <joints.yaml> <links.yaml>
//                   [vx vy wz ticks swing stance]
//
// Prints one line per 5 ms tick: 12 joints in CHAMP order LF RF LH RH
// (hip, upper, lower), i.e. the joints_map order. With ticks<=0 prints only
// the standing pose and the standing foot positions in the base frame.
// swing/stance <= 0 (or omitted) use the gait yaml values.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include <leg_controller/leg_controller.h>
#include <quadruped_base/quadruped_base.h>

#include "champ_robot_config.h"

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::fprintf(
      stderr,
      "usage: %s <urdf> <gait.yaml> <joints.yaml> <links.yaml> [vx vy wz ticks swing stance]\n",
      argv[0]);
    return 2;
  }
  champ_tools::RobotConfig cfg;
  try {
    cfg = champ_tools::loadRobotConfig(argv[1], argv[2], argv[3], argv[4]);
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "%s\n", ex.what());
    return 2;
  }

  const float vx = (argc > 5) ? static_cast<float>(std::atof(argv[5])) : 0.0f;
  const float vy = (argc > 6) ? static_cast<float>(std::atof(argv[6])) : 0.0f;
  const float wz = (argc > 7) ? static_cast<float>(std::atof(argv[7])) : 0.0f;
  const int ticks = (argc > 8) ? std::atoi(argv[8]) : 0;
  const float swing = (argc > 9) ? static_cast<float>(std::atof(argv[9])) : -1.0f;
  const float stance = (argc > 10) ? static_cast<float>(std::atof(argv[10])) : -1.0f;

  champ::GaitConfig gait = cfg.champGait();
  if (swing > 0.0f) {
    gait.swing_height = swing;
  }
  if (stance > 0.0f) {
    gait.stance_duration = stance;
  }
  // The dump replays whatever velocity is asked for; the nodes clamp cmd_vel to
  // the yaml limits themselves, so do not clamp here.
  gait.max_linear_velocity_x = std::max(gait.max_linear_velocity_x, std::fabs(vx));
  gait.max_linear_velocity_y = std::max(gait.max_linear_velocity_y, std::fabs(vy));
  gait.max_angular_velocity_z = std::max(gait.max_angular_velocity_z, std::fabs(wz));

  champ::QuadrupedBase base(gait);
  cfg.applyTo(base);

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
