#include <gtest/gtest.h>

#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"

// Exercise the actual production EKF update, not a reimplementation of it.
// A Euclidean manifold isolates linear measurement algebra from scan matching.
namespace {
using V30 = MTK::vect<30, double>;
MTK_BUILD_MANIFOLD(TestState, ((V30, values)));
using Filter = esekfom::esekf<TestState, 30, Eigen::Vector3d>;
using Cov = Eigen::Matrix<double, 30, 30>;
Eigen::MatrixXd observation;
Eigen::VectorXd measurement;
double variance;

Cov jacobian(TestState&, const Eigen::Vector3d&) { return Cov::Zero(); }
Eigen::Matrix<double, 30, 1> model(TestState&, const Eigen::Vector3d&) {
    return Eigen::Matrix<double, 30, 1>::Zero();
}
void measure(TestState& state, Eigen::Matrix3d, Eigen::Matrix3d,
             esekfom::dyn_share_modified<double>& data) {
    data.valid = observation.rows() > 0;
    data.h_x = observation.leftCols(12);
    data.z = measurement - observation * state.values;
    data.M_Noise = variance;
}

Cov prior() {
    Cov a;
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 30; ++j) {
            a(i, j) = .03 * std::sin(1. + i * 7. + j * 3.);
        }
    }
    return a * a.transpose() + .2 * Cov::Identity();
}
void observations(int rows) {
    observation = Eigen::MatrixXd::Zero(rows, 30);
    measurement.resize(rows);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < 12; ++j) {
            observation(i, j) = std::sin(.21 * (i + 1) * (j + 1));
        }
        measurement(i) = .1 * std::cos(i * .17);
    }
}
Filter filter(const Cov& covariance) {
    Filter result;
    result.init_dyn_share_modified_2h(model, jacobian, measure);
    TestState state;
    state.values.setZero();
    result.change_x(state);
    Cov copy = covariance;
    result.change_P(copy);
    return result;
}

class KalmanRows : public ::testing::TestWithParam<int> {};
TEST_P(KalmanRows, MatchesInformationFormAndPreservesCovariance) {
    const Cov p = prior();
    observations(GetParam());
    variance = .005;  // Same measurement variance as the simulation config.
    const Cov information = p.inverse() + observation.transpose() * observation / variance;
    const Cov expected_cov = information.ldlt().solve(Cov::Identity());
    const Eigen::VectorXd expected = expected_cov * observation.transpose() * measurement / variance;
    auto actual = filter(p);
    ASSERT_TRUE(actual.update_iterated_dyn_share_modified());
    EXPECT_EQ(actual.update_statistics().matched_rows, GetParam());
    EXPECT_EQ(actual.update_statistics().max_rows, GetParam());
    EXPECT_EQ(actual.update_statistics().batch_updates, GetParam() >= 30 ? 1 : 0);
    EXPECT_LT((actual.get_x().values - expected).norm(), 1e-9);
    EXPECT_LT((actual.get_P() - expected_cov).norm(), 1e-9);
    EXPECT_LT((actual.get_P() - actual.get_P().transpose()).norm(), 1e-9);
    Eigen::SelfAdjointEigenSolver<Cov> eig(actual.get_P());
    EXPECT_GT(eig.eigenvalues().minCoeff(), 0.);
}
INSTANTIATE_TEST_SUITE_P(SingleAndSnapshot, KalmanRows,
                        ::testing::Values(1, 10, 29, 30, 31, 100, 2000));

TEST(KalmanUpdate, SnapshotBatchEqualsSequentialSameTimeObservations) {
    const Cov p = prior();
    observations(100);
    const auto full_h = observation;
    const auto full_z = measurement;
    variance = .005;
    auto batch = filter(p);
    ASSERT_TRUE(batch.update_iterated_dyn_share_modified());
    auto sequential = filter(p);
    for (int i = 0; i < full_h.rows(); ++i) {
        observation = full_h.row(i);
        measurement = full_z.segment(i, 1);
        ASSERT_TRUE(sequential.update_iterated_dyn_share_modified());
    }
    EXPECT_LT((batch.get_x().values - sequential.get_x().values).norm(), 1e-9);
    EXPECT_LT((batch.get_P() - sequential.get_P()).norm(), 1e-9);
}

TEST(KalmanUpdate, MissingMatchesDoesNotApplyMeasurementCorrection) {
    auto actual = filter(prior());
    const auto before_x = actual.get_x().values;
    const auto before_p = actual.get_P();
    observations(0);
    variance = .005;
    EXPECT_FALSE(actual.update_iterated_dyn_share_modified());
    EXPECT_EQ(actual.update_statistics().missing_matches, 1);
    EXPECT_EQ((actual.get_x().values - before_x).norm(), 0.);
    EXPECT_EQ((actual.get_P() - before_p).norm(), 0.);
}
}  // namespace
