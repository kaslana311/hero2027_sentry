// Copyright 2026 HERO contributors
// SPDX-License-Identifier: MIT
#ifndef PB_MINCO_SMOOTHER__WARM_START_BOUNDARY_HPP_
#define PB_MINCO_SMOOTHER__WARM_START_BOUNDARY_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

namespace pb_minco {

// Qualifies only the smoother's initial PVA. It neither issues recovery
// commands nor changes the MPC model. All vectors are in the map frame.
class WarmStartBoundaryGuard {
 public:
  struct Config {
    double feedback_timeout = 0.5;
    double stationary_speed = 0.03;
    double stationary_displacement = 0.02;
    double stationary_duration = 0.5;
    double projected_motion_speed = 0.10;
    double projected_motion_acceleration = 0.50;
    double velocity_mismatch = 0.20;
    double mismatch_duration = 0.5;
    double acceleration_filter_time = 0.2;
  };

  struct Boundary {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
    bool valid = false;
  };

  struct Observation : Boundary {
    double stamp = 0.0;
    bool stationary = false;
  };

  enum class Reason {
    Projection, NoProjection, NoFeedback, InvalidProjection,
    StationaryMismatch, VelocityMismatch
  };

  struct Selection {
    Boundary boundary;
    bool use_projection = false;
    Reason reason = Reason::NoFeedback;
  };

  WarmStartBoundaryGuard() = default;
  explicit WarmStartBoundaryGuard(const Config &config) : config_(config) {}

  void reset() {
    observation_ = Observation{};
    stationary_since_ = 0.0;
    stationary_anchor_.setZero();
    mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool observe(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity,
               double stamp, double now, double max_acceleration) {
    if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(stamp) ||
        !std::isfinite(now) || stamp <= 0.0 || now - stamp < -0.05 ||
        now - stamp > config_.feedback_timeout) {
      return false;
    }
    // A reset simulation clock must not reuse a previous run's motion window.
    if (observation_.valid && now < observation_.stamp - config_.feedback_timeout) {
      reset();
    }
    if (observation_.valid && stamp <= observation_.stamp) {
      return false;
    }

    const double dt = stamp - observation_.stamp;
    const bool contiguous = observation_.valid && dt <= config_.feedback_timeout;
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
    if (contiguous && dt >= 1e-3) {
      Eigen::Vector2d derivative = (velocity - observation_.velocity) / dt;
      const double norm = derivative.norm();
      if (max_acceleration > 0.0 && norm > max_acceleration) {
        derivative *= max_acceleration / norm;
      }
      const double alpha = dt / (config_.acceleration_filter_time + dt);
      acceleration = observation_.acceleration +
        alpha * (derivative - observation_.acceleration);
    }

    if (!contiguous || velocity.norm() > config_.stationary_speed ||
        (position - stationary_anchor_).norm() > config_.stationary_displacement) {
      stationary_since_ = stamp;
      stationary_anchor_ = position;
    }
    if (!contiguous) {
      mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
    }
    observation_.position = position;
    observation_.velocity = velocity;
    observation_.acceleration = acceleration;
    observation_.stamp = stamp;
    observation_.stationary = velocity.norm() <= config_.stationary_speed &&
      stamp - stationary_since_ >= config_.stationary_duration;
    observation_.valid = true;
    return true;
  }

  Observation observation(double now) const {
    Observation result = observation_;
    const double age = now - result.stamp;
    result.valid = result.valid && std::isfinite(now) && age >= -0.05 &&
      age <= config_.feedback_timeout;
    return result;
  }

  Selection select(const Boundary &projection, double now, double max_distance) {
    Selection result;
    const auto measured = observation(now);
    if (!measured.valid) {
      mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
      return result;
    }

    // Fallback is anchored to the robot, not to the old reference/path start.
    result.boundary.position = measured.position;
    if (!measured.stationary) {
      result.boundary.velocity = measured.velocity;
      result.boundary.acceleration = measured.acceleration;
    }
    result.boundary.valid = true;
    result.reason = Reason::NoProjection;
    if (!projection.valid) {
      mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
      return result;
    }
    if (!projection.position.allFinite() || !projection.velocity.allFinite() ||
        !projection.acceleration.allFinite() || !std::isfinite(max_distance) ||
        max_distance <= 0.0 ||
        (projection.position - measured.position).norm() >= max_distance) {
      mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
      result.reason = Reason::InvalidProjection;
      return result;
    }

    Reason mismatch = Reason::Projection;
    if (measured.stationary &&
        (projection.velocity.norm() > config_.projected_motion_speed ||
         projection.acceleration.norm() > config_.projected_motion_acceleration)) {
      mismatch = Reason::StationaryMismatch;
    } else if ((projection.velocity - measured.velocity).norm() >
               config_.velocity_mismatch) {
      mismatch = Reason::VelocityMismatch;
    }
    if (mismatch != Reason::Projection) {
      // Do not reset on a single noisy frame or normal startup latency.
      if (!std::isfinite(mismatch_since_) || now < mismatch_since_) {
        mismatch_since_ = now;
      }
      if (now - mismatch_since_ >= config_.mismatch_duration) {
        result.reason = mismatch;
        return result;
      }
    } else {
      mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
    }
    // Healthy hot starts retain the original position, velocity AND acceleration.
    result.boundary = projection;
    result.use_projection = true;
    result.reason = Reason::Projection;
    return result;
  }

 private:
  Config config_;
  Observation observation_;
  Eigen::Vector2d stationary_anchor_ = Eigen::Vector2d::Zero();
  double stationary_since_ = 0.0;
  double mismatch_since_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace pb_minco
#endif  // PB_MINCO_SMOOTHER__WARM_START_BOUNDARY_HPP_
