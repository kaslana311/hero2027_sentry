// Copyright 2026 HERO contributors
// SPDX-License-Identifier: MIT
#include "pb_minco_smoother/warm_start_boundary.hpp"

#include "gcopter/minco.hpp"
#include <gtest/gtest.h>

#include <limits>

namespace {
using Guard = pb_minco::WarmStartBoundaryGuard;
using V2 = Eigen::Vector2d;

Guard::Boundary boundary(V2 position = V2::Zero(), V2 velocity = V2::Zero(),
                         V2 acceleration = V2::Zero()) {
  Guard::Boundary result;
  result.position = position;
  result.velocity = velocity;
  result.acceleration = acceleration;
  result.valid = true;
  return result;
}

void expectSame(const Guard::Boundary &actual, const Guard::Boundary &expected) {
  EXPECT_TRUE(actual.valid);
  EXPECT_TRUE(actual.position.isApprox(expected.position, 1e-12));
  EXPECT_TRUE(actual.velocity.isApprox(expected.velocity, 1e-12));
  EXPECT_TRUE(actual.acceleration.isApprox(expected.acceleration, 1e-12));
}

void stopped(Guard &guard) {
  for (double t : {1.0, 1.25, 1.5}) {
    ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), t, t, 2.0));
  }
  ASSERT_TRUE(guard.observation(1.5).stationary);
}

Trajectory<5, 2> polynomial(const Guard::Boundary &head) {
  Eigen::Matrix<double, 2, 3> head_pva, tail_pva;
  head_pva << head.position, head.velocity, head.acceleration;
  tail_pva.setZero();
  tail_pva.col(0) = V2(1.0, 0.0);
  minco::MINCO_S3NU solver;
  solver.setConditions(head_pva, tail_pva, 1);
  Eigen::Matrix2Xd inner(2, 0);
  Eigen::VectorXd times(1);
  times << 2.0;
  solver.setParameters(inner, times);
  Trajectory<5, 2> result;
  solver.getTrajectory(result);
  return result;
}

TEST(WarmStartBoundary, RequiresFreshFeedback) {
  Guard guard;
  EXPECT_FALSE(guard.select(boundary(), 1.0, 0.25).boundary.valid);
  ASSERT_TRUE(guard.observe(V2(1, 2), V2(0.3, 0), 1.0, 1.0, 2.0));
  EXPECT_FALSE(guard.select(boundary(), 1.501, 0.25).boundary.valid);
  EXPECT_FALSE(guard.observation(0.9).valid);
}

TEST(WarmStartBoundary, RejectsInvalidStaleFutureAndOutOfOrderObservations) {
  Guard guard;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(guard.observe(V2(nan, 0), V2::Zero(), 1, 1, 2));
  EXPECT_FALSE(guard.observe(V2::Zero(), V2(nan, 0), 1, 1, 2));
  EXPECT_FALSE(guard.observe(V2::Zero(), V2::Zero(), 0, 1, 2));
  EXPECT_FALSE(guard.observe(V2::Zero(), V2::Zero(), 1, 2, 2));
  EXPECT_FALSE(guard.observe(V2::Zero(), V2::Zero(), 2, 1, 2));
  ASSERT_TRUE(guard.observe(V2(2, 3), V2::Zero(), 2, 2, 2));
  EXPECT_FALSE(guard.observe(V2(9, 9), V2::Zero(), 1.9, 2, 2));
  EXPECT_FALSE(guard.observe(V2(9, 9), V2::Zero(), 2, 2, 2));
  EXPECT_TRUE(guard.observation(2).position.isApprox(V2(2, 3)));
}

TEST(WarmStartBoundary, HealthyCruisePreservesEntireProjectedPVA) {
  Guard guard;
  const auto reference = boundary(V2(0.04, 0.01), V2(0.6, 0.2), V2(0.3, -0.4));
  for (int i = 0; i < 40; ++i) {
    const double t = 1.0 + i * 0.125;
    ASSERT_TRUE(guard.observe(V2::Zero(), reference.velocity, t, t, 2));
    const auto result = guard.select(reference, t, 0.25);
    ASSERT_TRUE(result.use_projection);
    expectSame(result.boundary, reference);
  }
}

TEST(WarmStartBoundary, NormalAccelerationPreservesPlannedAcceleration) {
  Guard guard;
  for (int i = 0; i < 20; ++i) {
    const double t = 1.0 + i * 0.125;
    const V2 velocity(0.1 + i * 0.02, 0.04);
    ASSERT_TRUE(guard.observe(V2::Zero(), velocity, t, t, 2));
    const auto reference = boundary(V2::Zero(), velocity, V2(0.16, 0));
    const auto result = guard.select(reference, t, 0.25);
    ASSERT_TRUE(result.use_projection);
    expectSame(result.boundary, reference);
  }
}

TEST(WarmStartBoundary, LowSpeedLateralMotionIsNotBlanketZeroed) {
  Guard guard;
  for (int i = 0; i < 20; ++i) {
    const double t = 1.0 + i * 0.125;
    const V2 position(0, 0.02 * (t - 1));
    ASSERT_TRUE(guard.observe(position, V2(0, 0.02), t, t, 2));
    const auto reference = boundary(position, V2(0, 0.02), V2(0, 0.1));
    const auto result = guard.select(reference, t, 0.25);
    ASSERT_TRUE(result.use_projection);
    expectSame(result.boundary, reference);
  }
}

TEST(WarmStartBoundary, TransientMismatchDoesNotBreakContinuity) {
  Guard guard;
  stopped(guard);
  const auto ghost = boundary(V2::Zero(), V2(0.24, 0), V2(0.89, 0));
  EXPECT_TRUE(guard.select(ghost, 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 1.75, 1.75, 2));
  EXPECT_TRUE(guard.select(ghost, 1.75, 0.25).use_projection);
  const auto matching = boundary();
  EXPECT_TRUE(guard.select(matching, 1.75, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2, 2, 2));
  EXPECT_TRUE(guard.select(ghost, 2, 0.25).use_projection);
}

TEST(WarmStartBoundary, StationaryRobotDropsPersistentPhantomVelocityAndAcceleration) {
  Guard guard;
  stopped(guard);
  // Magnitudes observed in 19 consecutive replans while the robot was stuck.
  const auto ghost = boundary(V2(0.01, 0), V2(0.23622995772061803, 0),
                              V2(0.8904085286319303, 0));
  EXPECT_TRUE(guard.select(ghost, 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 1.75, 1.75, 2));
  EXPECT_TRUE(guard.select(ghost, 1.75, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2, 2, 2));
  const auto result = guard.select(ghost, 2, 0.25);
  EXPECT_FALSE(result.use_projection);
  EXPECT_EQ(result.reason, Guard::Reason::StationaryMismatch);
  expectSame(result.boundary, boundary());
}

TEST(WarmStartBoundary, StationaryAccelerationAloneCanInvalidateCache) {
  Guard guard;
  stopped(guard);
  const auto ghost = boundary(V2::Zero(), V2::Zero(), V2(0, 0.89));
  EXPECT_TRUE(guard.select(ghost, 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2, 2, 2));
  EXPECT_FALSE(guard.select(ghost, 2, 0.25).use_projection);
}

TEST(WarmStartBoundary, CacheRefreshCannotRestartMismatchWindow) {
  Guard guard;
  stopped(guard);
  bool rejected = false;
  for (int i = 0; i < 8; ++i) {
    const double t = 1.5 + i * 0.125;
    if (i > 0) {ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), t, t, 2));}
    // Represents a newly cached trajectory every replan, with the same bad PVA.
    const auto fresh_cache = boundary(V2::Zero(), V2(-0.236, 0), V2(-0.89, 0));
    rejected = rejected || !guard.select(fresh_cache, t, 0.25).use_projection;
  }
  EXPECT_TRUE(rejected);
}

TEST(WarmStartBoundary, SustainedMovingMismatchUsesActualNonzeroVelocity) {
  Guard guard;
  Guard::Selection result;
  for (int i = 0; i <= 4; ++i) {
    const double t = 1 + i * 0.125;
    const V2 position(-0.15 * (t - 1), 0);
    ASSERT_TRUE(guard.observe(position, V2(-0.15, 0), t, t, 2));
    result = guard.select(boundary(position, V2(0.24, 0)), t, 0.25);
  }
  EXPECT_FALSE(result.use_projection);
  EXPECT_EQ(result.reason, Guard::Reason::VelocityMismatch);
  EXPECT_TRUE(result.boundary.velocity.isApprox(V2(-0.15, 0)));
  EXPECT_TRUE(result.boundary.position.isApprox(V2(-0.075, 0)));
}

TEST(WarmStartBoundary, MissingFarOrNonfiniteProjectionUsesMeasuredPosition) {
  Guard guard;
  ASSERT_TRUE(guard.observe(V2(2, 3), V2(0.2, -0.1), 1, 1, 2));
  const auto expected = boundary(V2(2, 3), V2(0.2, -0.1));
  expectSame(guard.select(Guard::Boundary{}, 1, 0.25).boundary, expected);
  EXPECT_FALSE(guard.select(boundary(V2(8, 9)), 1, 0.25).use_projection);
  expectSame(guard.select(boundary(V2(8, 9)), 1, 0.25).boundary, expected);
  auto invalid = boundary(V2(2, 3));
  invalid.acceleration.x() = std::numeric_limits<double>::infinity();
  const auto result = guard.select(invalid, 1, 0.25);
  EXPECT_FALSE(result.use_projection);
  expectSame(result.boundary, expected);
}

TEST(WarmStartBoundary, FeedbackGapResetsStationaryAndMismatchWindows) {
  Guard guard;
  stopped(guard);
  EXPECT_TRUE(guard.select(boundary(V2::Zero(), V2(0.3, 0)), 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2.25, 2.25, 2));
  EXPECT_FALSE(guard.observation(2.25).stationary);
  EXPECT_TRUE(guard.select(boundary(V2::Zero(), V2(0.3, 0)), 2.25, 0.25).use_projection);
}

TEST(WarmStartBoundary, SimulationClockRollbackDoesNotKeepOldWindow) {
  Guard guard;
  ASSERT_TRUE(guard.observe(V2(9, 9), V2(1, 0), 10, 10, 2));
  ASSERT_TRUE(guard.observe(V2(1, 2), V2::Zero(), 1, 1, 2));
  EXPECT_FALSE(guard.observation(1).stationary);
  EXPECT_TRUE(guard.observation(1).position.isApprox(V2(1, 2)));
}

TEST(WarmStartBoundary, SmallOdometryNoiseStillRecognizesStationaryRobot) {
  Guard guard;
  for (int i = 0; i < 12; ++i) {
    const double t = 1 + i * 0.125;
    ASSERT_TRUE(guard.observe(V2(i % 2 * 0.001, 0), V2(0.007, 0), t, t, 2));
  }
  ASSERT_TRUE(guard.observation(2.375).stationary);
  const auto result = guard.select(Guard::Boundary{}, 2.375, 0.25);
  EXPECT_TRUE(result.boundary.velocity.isZero());
  EXPECT_TRUE(result.boundary.acceleration.isZero());
}

TEST(WarmStartBoundary, FallbackAccelerationIsFilteredAndBounded) {
  Guard guard;
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 1, 1, 2));
  ASSERT_TRUE(guard.observe(V2::Zero(), V2(1, 1), 1.125, 1.125, 2));
  const auto acceleration = guard.observation(1.125).acceleration;
  EXPECT_TRUE(acceleration.allFinite());
  EXPECT_GT(acceleration.norm(), 0);
  EXPECT_LT(acceleration.norm(), 2);
}

TEST(WarmStartBoundary, HealthyHotStartResumesAfterBoundaryReset) {
  Guard guard;
  stopped(guard);
  const auto ghost = boundary(V2::Zero(), V2(0.3, 0));
  EXPECT_TRUE(guard.select(ghost, 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2, 2, 2));
  EXPECT_FALSE(guard.select(ghost, 2, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2(0.01, 0), V2(0.1, 0), 2.125, 2.125, 2));
  const auto reference = boundary(V2(0.01, 0), V2(0.1, 0), V2(0.2, 0));
  const auto result = guard.select(reference, 2.125, 0.25);
  EXPECT_TRUE(result.use_projection);
  expectSame(result.boundary, reference);
}

TEST(MincoBoundaryRegression, PhantomReverseBoundaryForcesWrongWayThenGuardRemovesIt) {
  Guard guard;
  stopped(guard);
  const auto ghost = boundary(V2::Zero(), V2(-0.236, 0), V2(-0.89, 0));
  const auto before = polynomial(ghost);
  EXPECT_LT(before.getPos(0.05).x(), 0);
  EXPECT_NEAR(before.getVel(0).x(), -0.236, 1e-10);
  EXPECT_TRUE(guard.select(ghost, 1.5, 0.25).use_projection);
  ASSERT_TRUE(guard.observe(V2::Zero(), V2::Zero(), 2, 2, 2));
  const auto selected = guard.select(ghost, 2, 0.25);
  ASSERT_FALSE(selected.use_projection);
  const auto after = polynomial(selected.boundary);
  EXPECT_TRUE(after.getVel(0).isZero(1e-10));
  EXPECT_TRUE(after.getAcc(0).isZero(1e-10));
  for (int i = 1; i <= 200; ++i) {
    EXPECT_GE(after.getPos(i * 0.01).x(), -1e-10);
  }
}

TEST(MincoBoundaryRegression, RealReverseMotionIsPreservedRatherThanInstantlyZeroed) {
  Guard guard;
  ASSERT_TRUE(guard.observe(V2::Zero(), V2(-0.236, 0), 1, 1, 2));
  const auto reference = boundary(V2::Zero(), V2(-0.236, 0), V2(0.1, 0));
  const auto selected = guard.select(reference, 1, 0.25);
  ASSERT_TRUE(selected.use_projection);
  expectSame(selected.boundary, reference);
  const auto result = polynomial(selected.boundary);
  EXPECT_LT(result.getPos(0.05).x(), 0);
  EXPECT_NEAR(result.getVel(0).x(), -0.236, 1e-10);
}
}  // namespace
