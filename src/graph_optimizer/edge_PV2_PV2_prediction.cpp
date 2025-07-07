/**
 * @file edge_PV2_PV2_prediction.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Binary Edge for PV prediction interface
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/edge_PV2_PV2_prediction.hpp"

EdgePV2PV2Prediction::EdgePV2PV2Prediction():
    g2o::BaseBinaryEdge<PV2_STATE_SIZE, g2o::VectorN<PV2_STATE_SIZE>, VertexPV2Radar, VertexPV2Radar>() {
    setInformation(g2o::MatrixN<PV2_STATE_SIZE>::Identity());
}

bool EdgePV2PV2Prediction::read(std::istream& is) {
    return true;
}

bool EdgePV2PV2Prediction::write(std::ostream& os) const {
    return true;
}

void EdgePV2PV2Prediction::computeError() {
    const VertexPV2Radar* v1 = static_cast<const VertexPV2Radar*>(vertex(0));
    const VertexPV2Radar* v2 = static_cast<const VertexPV2Radar*>(vertex(1));

    g2o::VectorN<PV2_STATE_SIZE> v1_estimate = v1->estimateVec();
    g2o::VectorN<PV2_STATE_SIZE> v2_estimate = v2->estimateVec();

    double dt = (v2->timestamp - v1->timestamp);
    if ( dt < FLT_MIN ) {
        dt = 0.1;
    }

    // --- Mid-point Integration with Robust Error Calculation ---
    g2o::VectorN<PV2_STATE_SIZE> v1_mid_prior = Prediction(v1_estimate, dt*0.5);
    g2o::VectorN<PV2_STATE_SIZE> v2_mid_prior = Prediction(v2_estimate, -dt*0.5);

    // 1. Extract 2D orientation angles from the two mid-point states
    double theta1 = v1_mid_prior[IDX2_YAW];  // 2D orientation angle for first state
    double theta2 = v2_mid_prior[IDX2_YAW];  // 2D orientation angle for second state
    // 2. Build the error vector component-wise
    // Positional error
    _error.segment<2>(IDX2_X) = v2_mid_prior.segment<2>(IDX2_X) - v1_mid_prior.segment<2>(IDX2_X);
    // Linear velocity error
    _error.segment<2>(IDX2_VX) = v2_mid_prior.segment<2>(IDX2_VX) - v1_mid_prior.segment<2>(IDX2_VX);
    // Rotational error
    // Normalize angle difference to [-π, π]
    double angle_diff = theta2 - theta1;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    _error[IDX2_YAW] = angle_diff;
    // Angular velocity error
    _error[IDX2_VYAW] = v2_mid_prior[IDX2_VYAW] - v1_mid_prior[IDX2_VYAW];
}

g2o::VectorN<PV2_STATE_SIZE> EdgePV2PV2Prediction::Prediction(const g2o::VectorN<PV2_STATE_SIZE>& state, double dt) {
    g2o::VectorN<PV2_STATE_SIZE> prior;

    // Get current orientation angle from state vector
    double current_yaw = state[IDX2_YAW];

    // Linear prediction (assuming velocity is in the body frame)
    // 1. Get body-frame velocity
    Eigen::Vector2d v_body(state[IDX2_VX], state[IDX2_VY]);
    // 2. Rotate body-frame velocity to world frame using 2D rotation matrix
    Eigen::Matrix2d R_body_to_world;
    R_body_to_world << std::cos(current_yaw), -std::sin(current_yaw),
                       std::sin(current_yaw),  std::cos(current_yaw);
    Eigen::Vector2d v_world = R_body_to_world * v_body;
    // 3. Predict next position in world frame
    prior[IDX2_X]  = state[IDX2_X] + v_world.x() * dt;
    prior[IDX2_Y]  = state[IDX2_Y] + v_world.y() * dt;
    // 4. Assume constant velocity in body frame
    prior[IDX2_VX] = state[IDX2_VX];
    prior[IDX2_VY] = state[IDX2_VY];

    // Angular prediction (angular velocity is already in body frame)
    // 1. Update orientation with angular velocity
    prior[IDX2_YAW] = current_yaw + state[IDX2_VYAW] * dt;
    
    // 2. Normalize angle to [-π, π]
    while (prior[IDX2_YAW] > M_PI) prior[IDX2_YAW] -= 2.0 * M_PI;
    while (prior[IDX2_YAW] < -M_PI) prior[IDX2_YAW] += 2.0 * M_PI;

    // 3. Assume constant angular velocity in body frame
    prior[IDX2_VYAW] = state[IDX2_VYAW];

    return prior;
}

void EdgePV2PV2Prediction::setMeasurement(const g2o::Isometry2& measurement) {
    g2o::VectorN<PV2_STATE_SIZE> meas_vector;
    meas_vector.setZero();
    meas_vector[IDX2_X]  = measurement.translation().x();
    meas_vector[IDX2_Y]  = measurement.translation().y();
    meas_vector[IDX2_VX] = measurement.translation().x();
    meas_vector[IDX2_VY] = measurement.translation().y();
    meas_vector[IDX2_YAW] = std::atan2(measurement.rotation()(1, 0), measurement.rotation()(0, 0));
    meas_vector[IDX2_VYAW] = 0.0;
    setMeasurement(meas_vector);
}

void EdgePV2PV2Prediction::setMeasurement(const g2o::VectorN<PV2_STATE_SIZE>& measurement) {
    _measurement = measurement;
}

g2o::Isometry2 EdgePV2PV2Prediction::measurement() const {
    g2o::VectorN<PV2_STATE_SIZE> vec = _measurement;
    g2o::Isometry2 measurement;
    measurement.translate(Eigen::Vector2d(vec[IDX2_X], vec[IDX2_Y]));
    measurement.rotate(Eigen::Rotation2Dd(vec[IDX2_YAW]));
    return measurement;
}

void EdgePV2PV2Prediction::setInformation(const g2o::MatrixN<PV2_STATE_SIZE>& information) {
    _information = information;
}

void EdgePV2PV2Prediction::setInformation(const std::vector<double>& information) {
    double s_x, s_y, s_vx, s_vy, s_yaw, s_v_yaw;
    if ( information.size() == 6 ) {
        s_x  = information[0];
        s_y  = information[1];
        s_vx = 1.0;
        s_vy = 1.0;
        s_yaw = information[5];
        s_v_yaw = 1.0;
    }
    else if ( information.size() == 12 ) {
        s_x  = information[0];
        s_y  = information[1];
        s_vx = information[3];
        s_vy = information[4];
        s_yaw = information[8];
        s_v_yaw = information[11];
    }
    _information.setIdentity();
    information_wo_dt.setIdentity();
    information_wo_dt(0, 0)   = s_x;
    information_wo_dt(1, 1)   = s_y;
    information_wo_dt(2, 2)   = s_vx;
    information_wo_dt(3, 3)   = s_vy;
    information_wo_dt(4, 4)   = s_yaw;
    information_wo_dt(5, 5)   = s_v_yaw;
}

void EdgePV2PV2Prediction::getInformation(std::vector<double>& information) const {
    for ( int i = 0; i < PV2_STATE_SIZE; i++ ) {
        information[i] = _information(i, i);
    }
}

/**
 * @brief Returns 6x6 information matrix containing position and orientation covariances
 *        [0:2,0:2] - Position covariance (x,y)
 *        [3:5,3:5] - Orientation covariance (yaw, angular velocity) 
 *        [0:2,3:5] - Position-orientation cross covariance
 *        [3:5,0:2] - Position-orientation cross covariance (transpose)
  * 
 * @return g2o::MatrixN<6> 
 */
g2o::MatrixN<PV2_STATE_SIZE> EdgePV2PV2Prediction::informationM6() const {
    // Check if _information matrix is initialized and has correct size
    if (_information.rows() != PV2_STATE_SIZE || _information.cols() != PV2_STATE_SIZE) {
        std::cerr << "Error: _information matrix has invalid dimensions" << std::endl;
        return g2o::MatrixN<PV2_STATE_SIZE>::Identity(); // Return identity as fallback
    }

    return _information;
}
