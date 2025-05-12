/*
 * @copyright Automotive Intelligence Lab, Hanyang University
 * @author Jaeyoung Jo (wodud3743@gmail.com)
 * @file utils.hpp
 * @brief util functions for radar 4 vox map algorithm
 * @version 1.0
 * @date 2024-09-09
 */

#include "utils.hpp"


void CalFramePointCovTbb(std::vector<SRadarPoint> &points, double range_var_m, double azim_var_deg, double ele_var_deg) {
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                points[i] = CalPointCov(points[i], range_var_m, azim_var_deg, ele_var_deg);
            }
        }
    );
}

std::vector<SRadarPoint> ConvertRadarPointsToSRadarPointsTbb(const std::vector<RadarPoint>& radar_points, int i_frame_idx) {
    std::vector<SRadarPoint> s_radar_points;
    int i_radar_points_num = radar_points.size();
    s_radar_points.resize(i_radar_points_num);  // 벡터 크기를 미리 할당

    // TBB parallel_for를 사용하여 병렬로 작업 수행
    tbb::parallel_for(tbb::blocked_range<size_t>(0, i_radar_points_num),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const auto& radar_point = radar_points[i];
                auto& s_radar_point = s_radar_points[i];  // 직접 인덱스로 접근하여 수정

                s_radar_point.frame_idx = i_frame_idx;

                s_radar_point.pose = Eigen::Vector3d(radar_point.x, radar_point.y, radar_point.z);
                s_radar_point.cov = Eigen::Matrix4d::Identity();
                s_radar_point.local = Eigen::Vector3d(radar_point.x, radar_point.y, radar_point.z);

                s_radar_point.range = s_radar_point.pose.norm();
                s_radar_point.azi_angle = atan2(s_radar_point.pose.y(), s_radar_point.pose.x()) * 180.0 / M_PI;
                s_radar_point.ele_angle = atan2(s_radar_point.pose.z(),
                    sqrt(s_radar_point.pose.x() * s_radar_point.pose.x() +
                         s_radar_point.pose.y() * s_radar_point.pose.y())) * 180.0 / M_PI;

                s_radar_point.vel = radar_point.vel;
                s_radar_point.rcs = radar_point.rcs;
                s_radar_point.power = radar_point.power;

                // 필요한 경우 추가 필드 설정
                // s_radar_point.sensor_pose 등을 필요에 따라 설정
            }
        }
    );

    return s_radar_points;
}


std::vector<RadarPoint> ConvertSRadarPointsToRadarPointsTbb(const std::vector<SRadarPoint>& s_radar_points) {
    std::vector<RadarPoint> radar_points;
    int i_radar_points_num = s_radar_points.size();
    radar_points.resize(i_radar_points_num);  // 벡터 크기를 미리 할당

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, i_radar_points_num),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const auto& s_radar_point = s_radar_points[i];
                auto& radar_point = radar_points[i];  // 인덱스로 직접 접근

                radar_point.x = static_cast<float>(s_radar_point.pose.x());
                radar_point.y = static_cast<float>(s_radar_point.pose.y());
                radar_point.z = static_cast<float>(s_radar_point.pose.z());

                radar_point.range = static_cast<float>(s_radar_point.range);
                radar_point.azi = static_cast<float>(s_radar_point.azi_angle);
                radar_point.ele = static_cast<float>(s_radar_point.ele_angle);
                radar_point.vel = static_cast<float>(s_radar_point.vel);
                radar_point.rcs = static_cast<float>(s_radar_point.rcs);
                radar_point.power = static_cast<float>(s_radar_point.power);

                // 필요한 경우 추가 필드 설정
            }
        }
    );

    return radar_points;
}


std::pair<Eigen::MatrixXd, Eigen::VectorXd> getAandB(const std::vector<SRadarPoint>& points) {
    int n = points.size();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, 3);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
    if (n < 3) return {A, b}; // Not enough points

    for(int i = 0; i < n; ++i) {
        const auto& p = points[i];
        if (p.range > 1e-6) { // Avoid division by zero
            A.row(i) = p.pose.transpose() / p.range;
            b(i) = -p.vel;
        }
    }
    return {A, b};
}

/**
 * @brief Estimates local velocity from Doppler measurements using basic Least Squares.
 * 
 * This function provides a simple, non-iterative velocity estimate.
 * It does not perform outlier rejection.
 * 
 * @param points Radar points with Doppler measurements.
 * @return Estimated local velocity vector (Eigen::Vector3d). Returns zero vector if estimation fails.
 */
Eigen::Vector3d getVelocityFromDoppler(const std::vector<SRadarPoint>& points, Eigen::MatrixXd A, Eigen::VectorXd b) {
    if (A.rows() < 3) return Eigen::Vector3d::Zero();
    // Solve Ax = b using least squares (A^T * A * x = A^T * b)
    Eigen::Matrix3d AtA = A.transpose() * A;
    Eigen::Vector3d AtB = A.transpose() * b;
    // Could use QR decomposition for more stability: return A.colPivHouseholderQr().solve(b);
    return AtA.ldlt().solve(AtB); // Use LDLT for symmetric positive semi-definite
}

std::vector<int> getInlierIndices(const Eigen::Vector3d& estimated_velocity, const std::vector<SRadarPoint>& points, const Eigen::MatrixXd& A, const double vel_threshold) {
    std::vector<int> inliers;
    if (static_cast<size_t>(A.rows()) != points.size()) return inliers; // Use size_t cast

    for(size_t i = 0; i < points.size(); ++i) { // Use size_t for loop
        if (points[i].pose.norm() < 1e-6) continue; // Avoid normalizing zero vector
        double expected_doppler = points[i].pose.normalized().dot(estimated_velocity);
        if (std::abs(points[i].vel - expected_doppler) < vel_threshold) {
            inliers.push_back(static_cast<int>(i)); // Cast index to int if needed
        }
    }
    return inliers;
}

/**
 * @brief Estimate local velocity from radar points using weighted least squares
 * @param points Vector of radar points
 * @param estimated_velocity Output parameter for estimated velocity
 * @param velocity_covariance Output parameter for velocity covariance
 * @return True if estimation was successful
 */
bool estimateLocalVelocityFromPoints(
    const std::vector<SRadarPoint>& points,
    Eigen::Vector3d& estimated_velocity,
    Eigen::Matrix3d& velocity_covariance
) {
    if (points.size() < 3) return false; // Need at least 3 points

    auto [A, b] = getAandB(points); // Use static helper
    estimated_velocity = getVelocityFromDoppler(points, A, b); // Use static helper

    // Add simple covariance estimation (could be improved)
    Eigen::VectorXd residuals = A * estimated_velocity - b;
    double variance = residuals.squaredNorm() / (points.size() - 3.0); // N-DOF
    Eigen::Matrix3d AtA = A.transpose() * A;
    if (AtA.determinant() < 1e-9) { // Check for singularity
        velocity_covariance = Eigen::Matrix3d::Identity() * 1e6; // High uncertainty
        return false;
    }
    velocity_covariance = variance * AtA.inverse();

    return true;
}