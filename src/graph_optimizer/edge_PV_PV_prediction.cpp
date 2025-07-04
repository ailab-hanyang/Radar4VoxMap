/**
 * @file edge_PV_PV_prediction.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Binary Edge for PV prediction interface
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/edge_PV_PV_prediction.hpp"

EdgePVPVPrediction::EdgePVPVPrediction():
    g2o::BaseBinaryEdge<PV_STATE_SIZE, g2o::VectorN<PV_STATE_SIZE>, VertexPVRadar, VertexPVRadar>() {
    setInformation(g2o::MatrixN<PV_STATE_SIZE>::Identity());
}

bool EdgePVPVPrediction::read(std::istream& is) {
    return true;
}

bool EdgePVPVPrediction::write(std::ostream& os) const {
    return true;
}

void EdgePVPVPrediction::computeError() {
    const VertexPVRadar* v1 = static_cast<const VertexPVRadar*>(vertex(0));
    const VertexPVRadar* v2 = static_cast<const VertexPVRadar*>(vertex(1));

    g2o::VectorN<PV_STATE_SIZE> v1_estimate = v1->estimateVec();
    g2o::VectorN<PV_STATE_SIZE> v2_estimate = v2->estimateVec();

    double dt = (v2->timestamp - v1->timestamp);
    if ( dt < FLT_MIN ) {
        dt = 0.1;
    }

    // --- Mid-point Integration with Robust Error Calculation ---
    g2o::VectorN<PV_STATE_SIZE> v1_mid_prior = Prediction(v1_estimate, dt*0.5);
    g2o::VectorN<PV_STATE_SIZE> v2_mid_prior = Prediction(v2_estimate, -dt*0.5);

    // 1. Robustly reconstruct quaternions for the two mid-point states
    double q1_norm_sq = v1_mid_prior.segment<3>(IDX_QX).squaredNorm();
    double qw_1 = (q1_norm_sq < 1.0) ? std::sqrt(1.0 - q1_norm_sq) : 0.0;
    Eigen::Quaterniond q1(qw_1, v1_mid_prior[IDX_QX], v1_mid_prior[IDX_QY], v1_mid_prior[IDX_QZ]);

    double q2_norm_sq = v2_mid_prior.segment<3>(IDX_QX).squaredNorm();
    double qw_2 = (q2_norm_sq < 1.0) ? std::sqrt(1.0 - q2_norm_sq) : 0.0;
    Eigen::Quaterniond q2(qw_2, v2_mid_prior[IDX_QX], v2_mid_prior[IDX_QY], v2_mid_prior[IDX_QZ]);
    
    // 2. Build the error vector component-wise
    // Positional error
    _error.segment<3>(IDX_X) = v2_mid_prior.segment<3>(IDX_X) - v1_mid_prior.segment<3>(IDX_X);
    // Linear velocity error
    _error.segment<3>(IDX_VX) = v2_mid_prior.segment<3>(IDX_VX) - v1_mid_prior.segment<3>(IDX_VX);
    // Angular velocity error
    _error.segment<3>(IDX_WX) = v2_mid_prior.segment<3>(IDX_WX) - v1_mid_prior.segment<3>(IDX_WX);

    // Rotational error
    Eigen::Quaterniond q_error_quat = q1.inverse() * q2;
    if (q_error_quat.w() < 0) {
        q_error_quat.coeffs() *= -1.0;
    }
    _error.segment<3>(IDX_QX) = 2.0 * q_error_quat.vec();

    if ( _information.isApprox(g2o::MatrixN<PV_STATE_SIZE>::Identity()) ) {
        _information = information_wo_dt / (dt * dt);
    }
}

g2o::VectorN<PV_STATE_SIZE> EdgePVPVPrediction::Prediction(const g2o::VectorN<PV_STATE_SIZE>& state, double dt) {
    g2o::VectorN<PV_STATE_SIZE> prior;

    // Reconstruct current orientation from state vector
    double             qw = std::sqrt(1 - state[IDX_QX] * state[IDX_QX] - state[IDX_QY] * state[IDX_QY] - state[IDX_QZ] * state[IDX_QZ]);
    Eigen::Quaterniond q_current(qw, state[IDX_QX], state[IDX_QY], state[IDX_QZ]);
    q_current.normalize();

    // Linear prediction (assuming velocity is in the body frame)
    // 1. Get body-frame velocity
    Eigen::Vector3d v_body(state[IDX_VX], state[IDX_VY], state[IDX_VZ]);
    // 2. Rotate body-frame velocity to world frame
    Eigen::Vector3d v_world = q_current * v_body; 
    // 3. Predict next position in world frame
    prior[IDX_X]  = state[IDX_X] + v_world.x() * dt;
    prior[IDX_Y]  = state[IDX_Y] + v_world.y() * dt;
    prior[IDX_Z]  = state[IDX_Z] + v_world.z() * dt;
    // 4. Assume constant velocity in body frame
    prior[IDX_VX] = state[IDX_VX];
    prior[IDX_VY] = state[IDX_VY];
    prior[IDX_VZ] = state[IDX_VZ];

    // Angular prediction (angular velocity is already in body frame)
    // 1. get angular velocity vector
    Eigen::Vector3d omega(state[IDX_WX], state[IDX_WY], state[IDX_WZ]);
    double          omega_norm = omega.norm();

    // 2. Calculate small rotation based on angular velocity
    Eigen::Quaterniond q_delta;
    double          half_angle = 0.5 * omega_norm * dt;
    Eigen::Vector3d axis       = omega.normalized();
    q_delta = Eigen::Quaterniond(std::cos(half_angle), std::sin(half_angle) * axis.x(), std::sin(half_angle) * axis.y(),
                                    std::sin(half_angle) * axis.z());

    // 3. update new orientation
    Eigen::Quaterniond q_new = q_current * q_delta;
    q_new.normalize(); // normalize to maintain numerical stability

    prior[IDX_QX] = q_new.x();
    prior[IDX_QY] = q_new.y();
    prior[IDX_QZ] = q_new.z();
    // quaternion w is not calculated, so it is skipped

    // 4. Assume constant angular velocity in body frame
    prior[IDX_WX] = state[IDX_WX];
    prior[IDX_WY] = state[IDX_WY];
    prior[IDX_WZ] = state[IDX_WZ];

    return prior;
}

void EdgePVPVPrediction::setMeasurement(const g2o::Isometry3& measurement) {
    g2o::VectorN<PV_STATE_SIZE> meas_vector;
    meas_vector.setZero();
    g2o::Vector6 vec    = g2o::internal::toVectorMQT(measurement);
    meas_vector[IDX_X]  = vec[0];
    meas_vector[IDX_Y]  = vec[1];
    meas_vector[IDX_Z]  = vec[2];
    meas_vector[IDX_QX] = vec[3];
    meas_vector[IDX_QY] = vec[4];
    meas_vector[IDX_QZ] = vec[5];
    setMeasurement(meas_vector);
}

void EdgePVPVPrediction::setMeasurement(const g2o::VectorN<PV_STATE_SIZE>& measurement) {
    _measurement = measurement;
}

g2o::Isometry3 EdgePVPVPrediction::measurement() const {
    g2o::Vector6 vec;
    vec << _measurement[IDX_X], _measurement[IDX_Y], _measurement[IDX_Z], _measurement[IDX_QX], _measurement[IDX_QY], _measurement[IDX_QZ];
    g2o::Isometry3 measurement = g2o::internal::fromVectorMQT(vec);
    return measurement;
}

void EdgePVPVPrediction::setInformation(const g2o::MatrixN<PV_STATE_SIZE>& information) {
    _information = information;
}

void EdgePVPVPrediction::setInformation(const std::vector<double>& information) {
    double s_x, s_y, s_z, s_vx, s_vy, s_vz, s_qx, s_qy, s_qz, s_wx, s_wy, s_wz;
    if ( information.size() == 6 ) {
        s_x  = information[0];
        s_y  = information[1];
        s_z  = information[2];
        s_vx = 0.0001;
        s_vy = 0.0001;
        s_vz = 0.0001;
        s_qx = information[3];
        s_qy = information[4];
        s_qz = information[5];
        s_wx = 0.0001;
        s_wy = 0.0001;
        s_wz = 0.0001;
    }
    else if ( information.size() == 12 ) {
        s_x  = information[0];
        s_y  = information[1];
        s_z  = information[2];
        s_vx = information[3];
        s_vy = information[4];
        s_vz = information[5];
        s_qx = information[6];
        s_qy = information[7];
        s_qz = information[8];
        s_wx = information[9];
        s_wy = information[10];
        s_wz = information[11];
    }
    _information.setIdentity();
    information_wo_dt.setIdentity();
    information_wo_dt(0, 0)   = s_x;
    information_wo_dt(1, 1)   = s_y;
    information_wo_dt(2, 2)   = s_z;
    information_wo_dt(3, 3)   = s_vx;
    information_wo_dt(4, 4)   = s_vy;
    information_wo_dt(5, 5)   = s_vz;
    information_wo_dt(6, 6)   = s_qx;
    information_wo_dt(7, 7)   = s_qy;
    information_wo_dt(8, 8)   = s_qz;
    information_wo_dt(9, 9)   = s_wx;
    information_wo_dt(10, 10) = s_wy;
    information_wo_dt(11, 11) = s_wz;
}

void EdgePVPVPrediction::getInformation(std::vector<double>& information) const {
    for ( int i = 0; i < PV_STATE_SIZE; i++ ) {
        information[i] = _information(i, i);
    }
}

/**
 * @brief Returns 6x6 information matrix containing position and orientation covariances
 *        [0:2,0:2] - Position covariance (x,y,z)
 *        [3:5,3:5] - Orientation covariance (roll,pitch,yaw) 
 *        [0:2,3:5] - Position-orientation cross covariance
 *        [3:5,0:2] - Position-orientation cross covariance (transpose)
  * 
 * @return g2o::MatrixN<6> 
 */
g2o::MatrixN<6> EdgePVPVPrediction::informationM6() const {
    // Check if _information matrix is initialized and has correct size
    if (_information.rows() != 12 || _information.cols() != 12) {
        std::cerr << "Error: _information matrix has invalid dimensions" << std::endl;
        return g2o::MatrixN<6>::Identity(); // Return identity as fallback
    }

    g2o::MatrixN<6> information;
    information.setIdentity();

    // Extract position and orientation blocks from _information matrix
    // Top left 3x3: Position covariance
    information.topLeftCorner<3, 3>() = _information.topLeftCorner<3, 3>();
    
    // Bottom left 3x3: Position-orientation cross covariance 
    information.block<3, 3>(3, 0) = _information.block<3, 3>(6, 0);
    
    // Top right 3x3: Position-orientation cross covariance
    information.block<3, 3>(0, 3) = _information.block<3, 3>(0, 6);
    
    // Bottom right 3x3: Orientation covariance
    information.block<3, 3>(3, 3) = _information.block<3, 3>(6, 6);

    // Check if resulting information matrix is positive definite
    Eigen::LLT<Eigen::MatrixXd> lltOfInformation(information);
    if(lltOfInformation.info() == Eigen::NumericalIssue) {
        std::cerr << "Warning: Information matrix is not positive definite" << std::endl;
    }

    return information;
}
