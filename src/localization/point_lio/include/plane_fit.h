#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>

namespace point_lio {

// Orthogonal plane fit n.dot(p) + d = 0, with unit n and a free offset d.
// Solving A*n = -1 instead makes the result depend on the map origin and
// cannot represent a plane through that origin. Center before fitting, using
// double precision for the small local differences of world coordinates.
template <typename T, int N>
bool fit_plane(const Eigen::Matrix<T, N, 3>& points, T threshold,
               Eigen::Matrix<T, 4, 1>& plane) {
    if (points.rows() < 3 || !points.allFinite() ||
        !std::isfinite(threshold) || threshold < T(0)) return false;
    const Eigen::Matrix<double, N, 3> data = points.template cast<double>();
    const Eigen::Vector3d center = data.colwise().mean();
    const Eigen::Matrix<double, N, 3> centered = data.rowwise() - center.transpose();
    const Eigen::Matrix3d scatter = centered.transpose() * centered;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
    if (solver.info() != Eigen::Success) return false;
    const auto& eigenvalues = solver.eigenvalues();
    // A line or a coincident cluster has no uniquely observable plane normal.
    if (eigenvalues[2] <= 0. || eigenvalues[1] <= 1e-6 * eigenvalues[2]) return false;
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if ((centered * normal).cwiseAbs().maxCoeff() > threshold) return false;
    plane.template head<3>() = normal.template cast<T>();
    plane[3] = static_cast<T>(-normal.dot(center));
    return plane.allFinite();
}

}  // namespace point_lio
