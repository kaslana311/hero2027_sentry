#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "pb_minco_smoother/minco_optimizer.hpp"

namespace {

class SlantedWall : public pb_minco::ESDFInterface {
 public:
    double getDistance(double x, double y) const override {
        return 0.8 * x + 0.6 * y;
    }
    Eigen::Vector2d getGradient(double, double) const override {
        return {0.8, 0.6};
    }
    bool isInside(double, double) const override { return true; }
};

Trajectory<5, 2> run(const pb_minco::MincoOptimizerConfig &config,
                     bool with_obstacle = true) {
    pb_minco::MincoOptimizer optimizer;
    optimizer.setConfig(config);
    if (with_obstacle) {
        optimizer.setESDFInterface(std::make_shared<SlantedWall>());
    }
    Eigen::Matrix<double, 2, 3> head, tail;
    head.setZero();
    tail.setZero();
    head(0, 0) = -1.0;
    tail(0, 0) = 1.0;
    Eigen::Matrix2Xd points(2, 2);
    points << -0.33, 0.33, 0.0, 0.0;
    Eigen::VectorXd times = Eigen::VectorXd::Constant(3, 1.0);
    optimizer.initialize(head, tail, 3);
    EXPECT_TRUE(optimizer.optimize(points, times));
    Trajectory<5, 2> trajectory;
    optimizer.getTrajectory(trajectory);
    for (int i = 0; i < trajectory.getPieceNum(); ++i) {
        EXPECT_TRUE(trajectory[i].getCoeffMat().allFinite());
        EXPECT_TRUE(std::isfinite(trajectory[i].getDuration()));
        EXPECT_GT(trajectory[i].getDuration(), 0.0);
    }
    return trajectory;
}

double difference(const Trajectory<5, 2> &a, const Trajectory<5, 2> &b) {
    double maximum = 0.0;
    EXPECT_EQ(a.getPieceNum(), b.getPieceNum());
    for (int i = 0; i < a.getPieceNum(); ++i) {
        maximum = std::max(maximum,
            (a[i].getCoeffMat() - b[i].getCoeffMat()).cwiseAbs().maxCoeff());
        maximum = std::max(maximum,
            std::abs(a[i].getDuration() - b[i].getDuration()));
    }
    return maximum;
}

TEST(EsdfGradientSwitch, DefaultIsExactlyExplicitLegacyMode) {
    pb_minco::MincoOptimizerConfig defaults;
    EXPECT_TRUE(defaults.project_esdf_gradient);
    auto explicit_legacy = defaults;
    explicit_legacy.project_esdf_gradient = true;
    EXPECT_DOUBLE_EQ(difference(run(defaults), run(explicit_legacy)), 0.0);
}

TEST(EsdfGradientSwitch, ChangesOptimizationWhenGradientHasTangentComponent) {
    pb_minco::MincoOptimizerConfig projected;
    auto full = projected;
    full.project_esdf_gradient = false;
    EXPECT_GT(difference(run(projected), run(full)), 1e-5);
}

TEST(EsdfGradientSwitch, NoEsdfLeavesEverythingElseIdentical) {
    pb_minco::MincoOptimizerConfig projected;
    auto full = projected;
    full.project_esdf_gradient = false;
    EXPECT_DOUBLE_EQ(difference(run(projected, false), run(full, false)), 0.0);
}

}  // namespace
