#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>
#include "plane_fit.h"

namespace {
using Points = Eigen::Matrix<double, 5, 3>;
Points patch() {
    Points points;
    points << -.1, -.1, 0., .1, -.1, 0., -.1, .1, 0., .1, .1, 0., 0., 0., 0.;
    return points;
}

TEST(PlaneFit, FitsPlaneThroughCoordinateOrigin) {
    Eigen::Vector4d plane;
    ASSERT_TRUE(point_lio::fit_plane(patch(), .001, plane));
    EXPECT_NEAR(std::abs(plane.z()), 1., 1e-12);
    EXPECT_NEAR(plane.w(), 0., 1e-12);
}

TEST(PlaneFit, NoisyPlaneDoesNotDependOnMapOrigin) {
    auto points = patch();
    points.col(2) << .002, -.001, .001, -.002, .001;
    Eigen::Vector4d base, moved;
    ASSERT_TRUE(point_lio::fit_plane(points, .01, base));
    const Eigen::Vector3d shift(10., -7., .4);
    const Points translated = points.rowwise() + shift.transpose();
    ASSERT_TRUE(point_lio::fit_plane(translated, .01, moved));
    if (base.head<3>().dot(moved.head<3>()) < 0.) moved = -moved;
    EXPECT_LT((base.head<3>() - moved.head<3>()).norm(), 1e-10);
    EXPECT_NEAR(moved.w(), base.w() - base.head<3>().dot(shift), 1e-10);
}

TEST(PlaneFit, RotatedPlaneRetainsPerpendicularNormal) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.7, Eigen::Vector3d(1., 2., 3.).normalized()).toRotationMatrix();
    const Points points = patch() * rotation.transpose();
    Eigen::Vector4d plane;
    ASSERT_TRUE(point_lio::fit_plane(points, .001, plane));
    EXPECT_NEAR(std::abs(plane.head<3>().dot(rotation.col(2))), 1., 1e-12);
}

TEST(PlaneFit, RejectsLineAndCoincidentPoints) {
    Points points = Points::Zero();
    Eigen::Vector4d plane;
    EXPECT_FALSE(point_lio::fit_plane(points, .1, plane));
    points.col(0) << -.2, -.1, 0., .1, .2;
    EXPECT_FALSE(point_lio::fit_plane(points, .1, plane));
}

TEST(PlaneFit, RetainsPointToPlaneDistanceGate) {
    auto points = patch();
    points(4, 2) = .1;
    Eigen::Vector4d plane;
    EXPECT_FALSE(point_lio::fit_plane(points, .01, plane));
}

TEST(PlaneFit, RejectsNonfiniteCoordinates) {
    auto points = patch();
    points(0, 1) = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector4d plane;
    EXPECT_FALSE(point_lio::fit_plane(points, .1, plane));
}
}  // namespace
