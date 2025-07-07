/**
 * @file edge_PV2_doppler.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for doppler measurement
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/edge_PV2_doppler.hpp"

EdgePV2Doppler::EdgePV2Doppler():
    g2o::BaseUnaryEdge<2, g2o::Vector2, VertexPV2Radar>() {
    _measurement.setZero();
    setInformation(g2o::MatrixN<2>::Identity());
}

void EdgePV2Doppler::computeError() {
    const VertexPV2Radar*        v1    = static_cast<const VertexPV2Radar*>(_vertices[0]);
    // Convert velocity vector to local coordinate
    g2o::VectorN<PV2_STATE_SIZE> state = v1->estimateVec();
    Eigen::Vector2d             vel_body(state[IDX2_VX], state[IDX2_VY]);

    _error = vel_body - _measurement;
}

void EdgePV2Doppler::setMeasurement(const g2o::Vector2& measurement) {
    _measurement = measurement;
}

void EdgePV2Doppler::getMeasurement(g2o::Vector2& measurement) const {
    measurement = _measurement;
}

void EdgePV2Doppler::setInformation(const g2o::MatrixN<2>& information) {
    _information = information;
}

void EdgePV2Doppler::setInformationScale(const double information) {
    _information      = information * information_from_points;
    information_scale = information;
}

void EdgePV2Doppler::getInformation(std::vector<double>& information) const {
    for ( int i = 0; i < 2; ++i ) {
        for ( int j = 0; j < 2; ++j ) {
            information.push_back(_information(i, j));
        }
    }
}

bool EdgePV2Doppler::setPointMeasurementAndInformation(const std::vector<SRadarPoint>& points) {

    // const VertexPV2Radar*        v1    = static_cast<const VertexPV2Radar*>(_vertices[0]);
    // // Convert velocity vector to local coordinate
    // g2o::VectorN<PV2_STATE_SIZE> state = v1->estimateVec();
    // Eigen::Vector2d             vel_body(state[IDX2_VX], state[IDX2_VY]);

    // 2D 버전
    std::vector<Eigen::Vector2d> A_data;
    std::vector<double>          b_data;

    // 사전 할당: 메모리 할당을 최적화하기 위해 reserve 사용
    A_data.reserve(points.size());
    b_data.reserve(points.size());

    for ( const auto& point : points ) {
        if ( point.is_static ) {
            // Normalize point by its range
            A_data.emplace_back(point.pose.head<2>() / point.range); // A(i, 0), A(i, 1)
            b_data.emplace_back(-point.vel);               // b(i)
        }
    }

    // calculate actual matrix size
    const size_t n = A_data.size();
    if ( n < 10 ) {
        return false;
    }

    // set A matrix and b vector to actual size
    Eigen::MatrixXd A(n, 2); // nx2 (x/range, y/range)
    Eigen::VectorXd b(n);    // nx1 each point's doppler velocity

    for ( size_t i = 0; i < n; ++i ) {
        A.row(i) = A_data[i];
        b(i)     = b_data[i];
    }

    // solve normal equation: A^T * A * coeffs = A^T * b
    const Eigen::MatrixXd ATA    = A.transpose() * A; // 2x2
    Eigen::Vector2d       coeffs = (ATA).ldlt().solve(A.transpose() * b);

    // save result to _measurement
    // std::cout << "coeffs " << coeffs.transpose() << std::endl;
    _measurement = g2o::Vector2(coeffs(0), coeffs(1));

    // calculate Sigma
    const Eigen::VectorXd error = A * coeffs - b; // error = (nx2) * (2x1) - (nx1) = nx1, difference between estimated velocity and measured velocity
    const Eigen::MatrixXd covariance =
            (error.transpose() * error).x() * (ATA).inverse() / (A.rows() - 2); // .x() is to ensure it is 1x1

    Eigen::Vector2d sigma_v_r = Eigen::Vector2d(covariance(0, 0), covariance(1, 1));

    // if(sigma_v_r.x() > 0.001 || sigma_v_r.y() > 0.001)
    //     return false;

    information_from_points = covariance.inverse().matrix();

    return true;
}

std::vector<SRadarPoint> EdgePV2Doppler::ExtractStaticPoints(const std::vector<SRadarPoint>& points, const Eigen::Vector2d& velocity) {
    // Calculate expected doppler velocity using given velocity and point position
    // Expected doppler = (velocity_vector · point_position_normalized)
    std::vector<double> expected_doppler(points.size());
    std::vector<double> doppler_residual(points.size());
    
    for ( size_t i = 0; i < points.size(); ++i ) {
        // Normalize point position by range
        Eigen::Vector2d point_normalized = points[i].pose.head<2>() / points[i].range;
        
        // Calculate expected doppler velocity: v · (p/|p|)
        // velocity is [vx, vy] and point_normalized is normalized position
        expected_doppler[i] = velocity.x() * point_normalized.x() + 
                             velocity.y() * point_normalized.y();
        
        // Calculate residual between measured and expected doppler
        doppler_residual[i] = points[i].vel - expected_doppler[i];
    }

    // Extract static points based on low doppler residual
    std::vector<SRadarPoint> static_points;
    const double residual_threshold = 0.5; // threshold for static point classification
    
    for ( size_t i = 0; i < points.size(); ++i ) {
        if ( std::abs(doppler_residual[i]) < residual_threshold ) {
            auto point = points[i];
            point.is_static = true;
            static_points.push_back(std::move(point));
        }
    }
    
    return static_points;
}