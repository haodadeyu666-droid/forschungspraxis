#include "injection_common.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "scurve.h"

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }

Eigen::Affine3d toAffine(const std::array<double, 16>& m) {
  return Eigen::Affine3d(Eigen::Matrix4d::Map(m.data()));  // column-major, like libfranka
}

std::array<double, 16> toArray(const Eigen::Affine3d& T) {
  std::array<double, 16> m{};
  Eigen::Matrix4d::Map(m.data()) = T.matrix();
  return m;
}

Eigen::Affine3d poseFromConfig(const InjectionConfig& c) {
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() << c.start_position[0], c.start_position[1], c.start_position[2];
  T.linear() = (Eigen::AngleAxisd(deg2rad(c.start_rpy_deg[2]), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(deg2rad(c.start_rpy_deg[1]), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(deg2rad(c.start_rpy_deg[0]), Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  return T;
}

void printPose(const std::string& label, const Eigen::Affine3d& T) {
  const Eigen::Vector3d p = T.translation();
  const Eigen::Vector3d ypr = T.rotation().eulerAngles(2, 1, 0);  // yaw, pitch, roll
  const Eigen::Vector3d z = T.rotation().col(2);
  std::cout << std::fixed << std::setprecision(4) << label << "\n"
            << "  start_position = {" << p.x() << ", " << p.y() << ", " << p.z() << "};\n"
            << std::setprecision(2) << "  start_rpy_deg  = {" << rad2deg(ypr[2]) << ", "
            << rad2deg(ypr[1]) << ", " << rad2deg(ypr[0]) << "};\n"
            << std::setprecision(3) << "  needle axis (TCP z) in base = [" << z.x() << ", "
            << z.y() << ", " << z.z() << "]\n";
  std::cout.unsetf(std::ios::floatfield);
}

// Returns false if the user typed 'q'.
bool waitForEnter(const std::string& prompt) {
  std::cout << prompt << " [Enter = go, q + Enter = quit] " << std::flush;
  std::string line;
  if (!std::getline(std::cin, line)) return false;
  return line.empty() || (line[0] != 'q' && line[0] != 'Q');
}

void configureRobot(franka::Robot& robot, const InjectionConfig& c) {
  // TCP = needle tip, `flange_to_tip` along the flange z axis, orientation = flange.
  // setEE() is relative to the NE frame configured in Desk (e.g. Franka Hand), so remove
  // that first: NE_T_EE = inv(F_T_NE) * F_T_EE.
  Eigen::Affine3d F_T_EE = Eigen::Affine3d::Identity();
  F_T_EE.translation().z() = c.flange_to_tip;
  const Eigen::Affine3d NE_T_EE = toAffine(robot.readOnce().F_T_NE).inverse() * F_T_EE;
  robot.setEE(toArray(NE_T_EE));
  // Values from the libfranka examples.
  robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
}

// ---------------------------------------------------------------------------------------
// Move the TCP to `target` with a CartesianPose generator.
// Position is interpolated linearly, orientation by slerp, both driven by one normalized
// S-curve lambda(t) in [0, 1]. The normalized limits are the tightest of the translational
// and rotational limits, so each part stays within its own velocity/acc/jerk limits.
// ---------------------------------------------------------------------------------------
void moveToPose(franka::Robot& robot, const Eigen::Affine3d& target, const InjectionConfig& c) {
  const Eigen::Vector3d p1 = target.translation();
  const Eigen::Quaterniond q1(target.rotation());

  Eigen::Vector3d p0;
  Eigen::Quaterniond q0;
  SCurveProfile lambda;
  double time = 0.0;

  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
    time += period.toSec();
    if (time == 0.0) {
      const Eigen::Affine3d start = toAffine(state.O_T_EE_c);
      p0 = start.translation();
      q0 = Eigen::Quaterniond(start.rotation()).normalized();

      const double L = (p1 - p0).norm();
      const double theta = q0.angularDistance(q1);
      constexpr double kInf = std::numeric_limits<double>::infinity();
      auto scaled = [](double limit, double dist) { return dist > 1e-9 ? limit / dist : kInf; };
      const double vn = std::min(scaled(c.move_lin_velocity, L), scaled(c.move_rot_velocity, theta));
      const double an = std::min(scaled(c.move_lin_acc, L), scaled(c.move_rot_acc, theta));
      const double jn = std::min(scaled(c.move_lin_jerk, L), scaled(c.move_rot_jerk, theta));
      if (std::isfinite(vn)) {
        lambda = SCurveProfile(1.0, vn, an, jn, /*require_target_velocity=*/false);
      }
    }

    const double s = lambda.duration() > 0.0 ? lambda.at(time).s / lambda.distance() : 1.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = p0 + s * (p1 - p0);
    T.linear() = q0.slerp(s, q1).toRotationMatrix();

    if (time >= lambda.duration()) {
      T.translation() = p1;
      T.linear() = q1.toRotationMatrix();
      return franka::MotionFinished(franka::CartesianPose(toArray(T)));
    }
    return franka::CartesianPose(toArray(T));
  });
}

// ---------------------------------------------------------------------------------------
// Injection / retraction along the TCP z axis with a CartesianVelocities generator.
// The axis is fixed at the start of the motion from the commanded orientation.
// ---------------------------------------------------------------------------------------
struct LogRow {
  int phase;  // 1 = insert, 2 = retract
  double t;
  std::array<double, 3> pos;       // measured TCP position in base frame [m]
  std::array<double, 4> quat;      // measured TCP orientation in base frame (qx, qy, qz, qw)
  std::array<double, 6> vel;       // measured TCP twist in base frame, J(q) * dq [m/s, rad/s]
  std::array<double, 6> vel_cmd;   // commanded TCP twist in base frame [m/s, rad/s]
  std::array<double, 6> wrench_O;  // estimated external wrench on TCP, base frame [N, Nm]
  std::array<double, 6> wrench_K;  // same wrench in the TCP (stiffness) frame [N, Nm]
  double s_cmd, v_cmd;             // S-curve: commanded insertion depth [m] and speed [m/s]
  double depth_meas;  // measured TCP displacement along the needle axis from insertion start [m]
};

const char* kLogHeader =
    "phase,time,"
    "x,y,z,qx,qy,qz,qw,"
    "vx,vy,vz,wx,wy,wz,"
    "vx_cmd,vy_cmd,vz_cmd,wx_cmd,wy_cmd,wz_cmd,"
    "F_x,F_y,F_z,T_x,T_y,T_z,"
    "F_x_tcp,F_y_tcp,F_z_tcp,T_x_tcp,T_y_tcp,T_z_tcp,"
    "s_cmd,v_cmd,depth_meas\n";

void moveAlongNeedle(franka::Robot& robot, const franka::Model& model, const SCurveProfile& profile,
                     double direction, int phase, const Eigen::Vector3d& insertion_start,
                     std::vector<LogRow>& log) {
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  double time = 0.0;
  const double depth_offset = phase == 1 ? 0.0 : profile.distance();  // s_cmd as insertion depth

  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianVelocities {
    time += period.toSec();
    if (time == 0.0) {
      axis << state.O_T_EE_c[8], state.O_T_EE_c[9], state.O_T_EE_c[10];
    }

    const SCurveProfile::Sample cmd = profile.at(time);
    const double v = direction * cmd.v;

    if (log.size() < log.capacity()) {  // never reallocate inside the control loop
      const Eigen::Affine3d T = toAffine(state.O_T_EE);
      const Eigen::Vector3d p = T.translation();
      const Eigen::Quaterniond q(T.rotation());
      const std::array<double, 42> J_arr = model.zeroJacobian(franka::Frame::kEndEffector, state);
      const Eigen::Matrix<double, 6, 1> twist =
          Eigen::Map<const Eigen::Matrix<double, 6, 7>>(J_arr.data()) *
          Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data());

      LogRow r;
      r.phase = phase;
      r.t = time;
      r.pos = {p.x(), p.y(), p.z()};
      r.quat = {q.x(), q.y(), q.z(), q.w()};
      for (int i = 0; i < 6; ++i) r.vel[i] = twist[i];
      r.vel_cmd = state.O_dP_EE_c;
      r.wrench_O = state.O_F_ext_hat_K;
      r.wrench_K = state.K_F_ext_hat_K;
      r.s_cmd = depth_offset + direction * cmd.s;
      r.v_cmd = v;
      r.depth_meas = (p - insertion_start).dot(axis);
      log.push_back(r);
    }

    const std::array<double, 6> vel{{axis.x() * v, axis.y() * v, axis.z() * v, 0.0, 0.0, 0.0}};
    if (time >= profile.duration()) {
      return franka::MotionFinished(franka::CartesianVelocities(std::array<double, 6>{}));
    }
    return franka::CartesianVelocities(vel);
  });
}

template <size_t N>
void writeValues(std::ofstream& f, const std::array<double, N>& a) {
  for (double x : a) f << ',' << x;
}

void writeLog(const std::string& path, const std::vector<LogRow>& log) {
  std::ofstream f(path);
  f << kLogHeader << std::setprecision(9);
  for (const auto& r : log) {
    f << r.phase << ',' << r.t;
    writeValues(f, r.pos);
    writeValues(f, r.quat);
    writeValues(f, r.vel);
    writeValues(f, r.vel_cmd);
    writeValues(f, r.wrench_O);
    writeValues(f, r.wrench_K);
    f << ',' << r.s_cmd << ',' << r.v_cmd << ',' << r.depth_meas << '\n';
  }
}

// First rep index whose file does not exist yet, so earlier data is never overwritten.
int nextFreeRep(const fs::path& folder) {
  int rep = 1;
  while (fs::exists(folder / ("rep" + std::to_string(rep) + ".csv"))) ++rep;
  return rep;
}

}  // namespace

int run_injection_program(const InjectionConfig& c, int argc, char** argv) {
  const std::string arg = argc > 1 ? argv[1] : "";
  if (!(c.flange_to_tip > 0.0) || !(c.depth > 0.0)) {
    std::cerr << "Config error: flange_to_tip and depth must be set (> 0) in " << c.program_name << "\n";
    return -1;
  }

  try {
    franka::Robot robot(c.robot_ip);
    configureRobot(robot, c);
    const franka::Model model = robot.loadModel();  // for measured Cartesian velocity

    if (arg == "--print-pose") {
      printPose("Current TCP pose (paste into the program to use it as start pose):",
                toAffine(robot.readOnce().O_T_EE));
      return 0;
    }

    // Plan the injection once; fails before any motion if the speed is not reachable.
    const SCurveProfile insert_profile(c.depth, c.insert_velocity, c.injection_acc, c.injection_jerk);
    const SCurveProfile retract_profile(c.depth, c.retract_velocity, c.injection_acc, c.injection_jerk);

    const std::string condition = arg.empty() ? "correct" : arg;
    fs::path root(c.output_root);
    if (root.is_relative()) root = fs::path(PROJECT_DIR) / root;
    const fs::path folder = root / condition / c.program_name;
    fs::create_directories(folder);

    const Eigen::Affine3d start_pose = poseFromConfig(c);
    const Eigen::Affine3d current = toAffine(robot.readOnce().O_T_EE);
    const double move_dist = (start_pose.translation() - current.translation()).norm();

    std::cout << "=== " << c.program_name << " ===\n"
              << "Flange to tip " << c.flange_to_tip * 1000 << " mm, depth " << c.depth * 1000 << " mm, insert " << c.insert_velocity * 1000
              << " mm/s (" << insert_profile.duration() << " s), retract "
              << c.retract_velocity * 1000 << " mm/s (" << retract_profile.duration() << " s)\n"
              << "Data folder: " << folder << "\n\n";
    printPose("Current TCP pose:", current);
    printPose("Start TCP pose:", start_pose);
    std::cout << "Distance to start: " << move_dist * 1000 << " mm\n";

    if (move_dist > c.max_start_move_distance) {
      std::cerr << "Start pose is more than " << c.max_start_move_distance
                << " m away. Jog the robot closer or check start_position.\n";
      return -1;
    }
    if (!waitForEnter("Move to start pose?")) return 0;

    for (int i = 0, rep = nextFreeRep(folder); i < c.repetitions; ++i, ++rep) {
      std::cout << "\n--- Repetition " << i + 1 << " / " << c.repetitions << " (file rep" << rep
                << ".csv) ---\n";

      // Re-approach the start pose every rep: removes drift from velocity integration.
      moveToPose(robot, start_pose, c);
      if (!waitForEnter("At start pose. Insert needle?")) break;

      std::vector<LogRow> log;
      log.reserve(static_cast<size_t>((insert_profile.duration() + retract_profile.duration()) * 1000) + 2000);
      const Eigen::Vector3d insertion_start = toAffine(robot.readOnce().O_T_EE).translation();
      const std::string file = (folder / ("rep" + std::to_string(rep) + ".csv")).string();

      try {
        moveAlongNeedle(robot, model, insert_profile, +1.0, 1, insertion_start, log);
        std::cout << "Inserted. Measured depth: " << log.back().depth_meas * 1000 << " mm\n";
        moveAlongNeedle(robot, model, retract_profile, -1.0, 2, insertion_start, log);
        std::cout << "Retracted. Residual: " << log.back().depth_meas * 1000 << " mm\n";
      } catch (...) {
        writeLog(file, log);  // keep what was recorded up to the error
        std::cerr << "Partial log saved: " << file << "\n";
        throw;
      }
      writeLog(file, log);
      std::cout << "Saved " << file << "\n";
    }

    moveToPose(robot, start_pose, c);
    std::cout << "\nDone.\n";
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n"
              << "The needle may still be inserted - check the robot before restarting.\n";
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
