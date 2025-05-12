/**
 * @file edge_PV_doppler.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for doppler measurement
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/edge_PV_doppler.hpp"

EdgePVDoppler::EdgePVDoppler():
    g2o::BaseUnaryEdge<3, g2o::Vector3, VertexPVRadar>() {
    _measurement.setZero();
    setInformation(g2o::MatrixN<3>::Identity());
}

void EdgePVDoppler::computeError() {
    const VertexPVRadar*        v1    = static_cast<const VertexPVRadar*>(_vertices[0]);
    // Convert velocity vector to local coordinate
    g2o::VectorN<PV_STATE_SIZE> state = v1->estimateVec();
    double                      qw    = (1 - state[IDX_QX] * state[IDX_QX] - state[IDX_QY] * state[IDX_QY] - state[IDX_QZ] * state[IDX_QZ]);
    Eigen::Quaterniond          q(qw, state[IDX_QX], state[IDX_QY], state[IDX_QZ]);
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix();
    Eigen::Vector3d             vel_body(state[IDX_VX], state[IDX_VY], state[IDX_VZ]);
    Eigen::Vector3d             vel_local = R.transpose() * vel_body; // R: vertex의 orientation, vel_body: vertex의 global 속도벡터

    _error = vel_local - _measurement;
}

void EdgePVDoppler::setMeasurement(const g2o::Vector3& measurement) {
    _measurement = measurement;
}

void EdgePVDoppler::getMeasurement(g2o::Vector3& measurement) const {
    measurement = _measurement;
}

void EdgePVDoppler::setInformation(const g2o::MatrixN<3>& information) {
    _information = information;
}

void EdgePVDoppler::setInformationScale(const double information) {
    _information      = information * information_from_points;
    information_scale = information;
}

void EdgePVDoppler::getInformation(std::vector<double>& information) const {
    for ( int i = 0; i < 3; ++i ) {
        for ( int j = 0; j < 3; ++j ) {
            information.push_back(_information(i, j));
        }
    }
}

bool EdgePVDoppler::setPointMeasurementAndInformation(const std::vector<SRadarPoint>& points) {
    // 3D 버전: is_static == true인 포인트만 처리
    std::vector<Eigen::Vector3d> A_data;
    std::vector<double>          b_data;

    // 사전 할당: 메모리 할당을 최적화하기 위해 reserve 사용
    A_data.reserve(points.size());
    b_data.reserve(points.size());

    for ( const auto& point : points ) {
        if ( point.is_static ) {
            // Normalize point by its range
            A_data.emplace_back(point.pose / point.range); // A(i, 0), A(i, 1), A(i, 2)
            b_data.emplace_back(-point.vel);               // b(i)
        }
    }

    // calculate actual matrix size
    const size_t n = A_data.size();
    if ( n < 10 ) {
        return false;
    }

    // set A matrix and b vector to actual size
    Eigen::MatrixXd A(n, 3); // nx3 (x/range, y/range, z/range)
    Eigen::VectorXd b(n);    // nx1 each point's doppler velocity

    for ( size_t i = 0; i < n; ++i ) {
        A.row(i) = A_data[i];
        b(i)     = b_data[i];
    }

    // solve normal equation: A^T * A * coeffs = A^T * b
    const Eigen::MatrixXd ATA    = A.transpose() * A; // 3x3
    Eigen::Vector3d       coeffs = (ATA).ldlt().solve(A.transpose() * b);

    // save result to _measurement
    _measurement = g2o::Vector3(coeffs(0), coeffs(1), coeffs(2));

    // calculate Sigma
    const Eigen::VectorXd error = A * coeffs - b; // error = (nx3) * (3x1) - (nx1) = nx1, difference between estimated velocity and measured velocity
    const Eigen::MatrixXd covariance =
            (error.transpose() * error).x() * (ATA).inverse() / (A.rows() - 3); // .x() is to ensure it is 1x1

    Eigen::Vector3d sigma_v_r = Eigen::Vector3d(covariance(0, 0), covariance(1, 1), covariance(2, 2));

    // if(sigma_v_r.x() > 0.001 || sigma_v_r.y() > 0.001 || sigma_v_r.z() > 0.001)
    //     return false;

    // std::cout << "sigma_v_r " << sigma_v_r.transpose() << std::endl;
    // std::cout << "Information " << covariance.inverse().matrix() << std::endl;
    // std::cout << "Information Normalized" << covariance.inverse().matrix().normalized() << std::endl;

    information_from_points = covariance.inverse().matrix();

    return true;
}
