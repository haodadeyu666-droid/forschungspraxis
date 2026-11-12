#pragma once

#include <array>
#include <string>

// Settings for one injection program. Everything except the per-program block (name,
// start pose, flange-to-tip offset, depth) is shared between all programs, so all experiments use identical motion settings.
struct InjectionConfig {
  // --- Set per program -------------------------------------------------------------------
  std::string program_name;              // also used as output sub-folder
  std::array<double, 3> start_position;  // needle tip (TCP) in base frame [m]
  std::array<double, 3> start_rpy_deg;   // TCP orientation in base frame, R = Rz(y)*Ry(p)*Rx(r) [deg]
  double flange_to_tip = 0.0;            // flange -> needle tip along the flange z axis [m], must be > 0
  double depth = 0.0;                    // insertion depth along the needle [m], must be > 0

  // --- Shared: robot / tool --------------------------------------------------------------
  std::string robot_ip = "192.168.3.122";

  // --- Shared: move to start pose (CartesianPose, S-curve) -------------------------------
  double move_lin_velocity = 0.05;  // [m/s]
  double move_lin_acc = 0.2;        // [m/s^2]
  double move_lin_jerk = 1.0;       // [m/s^3]
  double move_rot_velocity = 0.2;   // [rad/s]
  double move_rot_acc = 0.5;        // [rad/s^2]
  double move_rot_jerk = 2.0;       // [rad/s^3]
  double max_start_move_distance = 0.5;  // safety: refuse longer first moves [m]

  // --- Shared: injection along TCP z axis (CartesianVelocities, S-curve) -----------------
  double insert_velocity = 0.005;   // [m/s]  must be reachable within each program's depth
  double retract_velocity = 0.005;  // [m/s]
  double injection_acc = 0.05;      // [m/s^2]
  double injection_jerk = 0.5;      // [m/s^3]

  // --- Shared: experiment ----------------------------------------------------------------
  int repetitions = 30;
  std::string output_root = "Needle_experiment";
};

// Entry point used by every program's main().
//   <program>                 -> run experiment, condition "correct"
//   <program> <condition>     -> run experiment, data in Needle_experiment/<condition>/<program>
//   <program> --print-pose    -> print current TCP pose in config format and exit
int run_injection_program(const InjectionConfig& config, int argc, char** argv);
