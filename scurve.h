#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

// Jerk-limited (7-segment, "double S") point-to-point profile in 1D.
//
// Moves from s = 0 (at rest) to s = distance (at rest) with peak velocity v_target,
// |acceleration| <= a_max and |jerk| <= j_max.
//
// require_target_velocity = true  : v_target must be reached (cruise phase exists),
//                                   otherwise the constructor throws.
// require_target_velocity = false : if the distance is too short, the peak velocity is
//                                   reduced so the profile still ends exactly at `distance`.
//
// If a_max cannot be reached before the peak velocity (v * j_max < a_max^2), the
// acceleration phase degenerates to a triangle with peak sqrt(v * j_max).
class SCurveProfile {
 public:
  struct Sample {
    double s;  // position
    double v;  // velocity
    double a;  // acceleration
  };

  // Zero-length profile (duration 0).
  SCurveProfile() = default;

  SCurveProfile(double distance, double v_target, double a_max, double j_max,
                bool require_target_velocity = true)
      : D_(distance), V_(v_target), J_(j_max) {
    if (D_ < 0.0 || V_ <= 0.0 || a_max <= 0.0 || J_ <= 0.0) {
      throw std::invalid_argument("SCurveProfile: distance must be >= 0, limits > 0");
    }
    if (D_ == 0.0) {
      if (require_target_velocity) {
        throw std::invalid_argument("SCurveProfile: distance must be > 0");
      }
      V_ = 0.0;
      return;
    }

    if (D_ < V_ * accelTime(V_, a_max)) {
      if (require_target_velocity) {
        throw std::invalid_argument(
            "SCurveProfile: distance " + std::to_string(D_) + " is too short to reach velocity " +
            std::to_string(V_) + " (needs >= " + std::to_string(V_ * accelTime(V_, a_max)) + ")");
      }
      V_ = peakVelocityForDistance(D_, a_max);
    }

    setupAccelPhase(a_max);
    Tv_ = std::max(0.0, (D_ - V_ * Ta_) / V_);
    T_ = 2.0 * Ta_ + Tv_;
  }

  double duration() const { return T_; }
  double distance() const { return D_; }
  double peakVelocity() const { return V_; }

  Sample at(double t) const {
    if (t <= 0.0) return {0.0, 0.0, 0.0};
    if (t >= T_) return {D_, 0.0, 0.0};
    if (t < Ta_) return accel(t);
    if (t <= Ta_ + Tv_) return {V_ * Ta_ / 2.0 + V_ * (t - Ta_), V_, 0.0};
    // Deceleration mirrors acceleration in time.
    Sample m = accel(T_ - t);
    return {D_ - m.s, m.v, -m.a};
  }

 private:
  // Duration of the acceleration phase from 0 to v.
  double accelTime(double v, double a_max) const {
    return (v * J_ >= a_max * a_max) ? v / a_max + a_max / J_ : 2.0 * std::sqrt(v / J_);
  }

  // Peak velocity v such that accel + decel (no cruise) covers exactly distance d.
  double peakVelocityForDistance(double d, double a_max) const {
    const double d_boundary = 2.0 * a_max * a_max * a_max / (J_ * J_);  // a_max just reached
    if (d >= d_boundary) {
      // v^2/a + v*a/j = d
      const double r = a_max / J_;
      return 0.5 * a_max * (-r + std::sqrt(r * r + 4.0 * d / a_max));
    }
    // 2 * v^(3/2) / sqrt(j) = d
    return std::cbrt(d * d * J_ / 4.0);
  }

  void setupAccelPhase(double a_max) {
    if (V_ * J_ >= a_max * a_max) {
      A_ = a_max;
      Tj_ = A_ / J_;
      Ta_ = V_ / A_ + Tj_;
    } else {
      Tj_ = std::sqrt(V_ / J_);
      A_ = J_ * Tj_;
      Ta_ = 2.0 * Tj_;
    }
  }

  // Acceleration phase, tau in [0, Ta].
  Sample accel(double tau) const {
    if (tau < Tj_) {
      return {J_ * tau * tau * tau / 6.0, J_ * tau * tau / 2.0, J_ * tau};
    }
    if (tau < Ta_ - Tj_) {
      return {A_ / 6.0 * (3.0 * tau * tau - 3.0 * Tj_ * tau + Tj_ * Tj_),
              A_ * (tau - Tj_ / 2.0), A_};
    }
    double r = Ta_ - tau;
    return {V_ * Ta_ / 2.0 - V_ * r + J_ * r * r * r / 6.0, V_ - J_ * r * r / 2.0, J_ * r};
  }

  double D_ = 0.0, V_ = 0.0, J_ = 1.0;
  double A_ = 0.0;   // peak acceleration actually used
  double Tj_ = 0.0;  // duration of each jerk segment
  double Ta_ = 0.0;  // duration of the acceleration phase
  double Tv_ = 0.0;  // duration of the cruise phase
  double T_ = 0.0;   // total duration
};
