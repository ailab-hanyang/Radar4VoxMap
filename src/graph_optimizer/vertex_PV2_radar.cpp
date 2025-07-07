/**
 * @file vertex_PV2_radar.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief VertexPVRadar interface
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/vertex_PV2_radar.hpp"

VertexPV2Radar::VertexPV2Radar():
    g2o::BaseVertex<PV2_STATE_SIZE, g2o::VectorN<PV2_STATE_SIZE>>() {
    setToOriginImpl();
    updateCache();
}

void VertexPV2Radar::setToOriginImpl() {
    _estimate.setZero();
}

/**
 * update the position of this vertex. The update is in the form
 * (x,y,z,qx,qy,qz) whereas (x,y,z) represents the translational update
 * and (qx,qy,qz) corresponds to the respective elements. The missing
 * element qw of the quaternion is recovred by
 * || (qw,qx,qy,qz) || == 1 => qw = sqrt(1 - || (qx,qy,qz) ||
 */
void VertexPV2Radar::oplusImpl(const double* update) {
    /* Pose update */
    _estimate[IDX2_X] += update[IDX2_X];
    _estimate[IDX2_Y] += update[IDX2_Y];
    // Angular update for 2D case (yaw only)
    _estimate[IDX2_YAW] += update[IDX2_YAW];
    
    // Normalize yaw angle to [-pi, pi]
    while (_estimate[IDX2_YAW] > M_PI) {
        _estimate[IDX2_YAW] -= 2.0 * M_PI;
    }
    while (_estimate[IDX2_YAW] < -M_PI) {
        _estimate[IDX2_YAW] += 2.0 * M_PI;
    }
    
    // Velocity update
    _estimate[IDX2_VX] += update[IDX2_VX];
    _estimate[IDX2_VY] += update[IDX2_VY];
    _estimate[IDX2_VYAW] += update[IDX2_VYAW];
}

bool VertexPV2Radar::read(std::istream& is) {
    is >> _estimate[IDX2_X] >> _estimate[IDX2_Y] >> _estimate[IDX2_VX] >> _estimate[IDX2_VY] >> _estimate[IDX2_YAW] >> _estimate[IDX2_VYAW];
    return true;
}

bool VertexPV2Radar::write(std::ostream& os) const {
    os << _estimate[IDX2_X] << " " << _estimate[IDX2_Y] << " " << _estimate[IDX2_VX] << " " << _estimate[IDX2_VY] << " " << _estimate[IDX2_YAW] << " " << _estimate[IDX2_VYAW];
    return os.good();
}

g2o::Isometry2 VertexPV2Radar::estimateIso2() const {
    g2o::VectorN<PV2_STATE_SIZE> est  = _estimate;
    g2o::Vector2                t    = {est[IDX2_X], est[IDX2_Y]};
    g2o::Isometry2              iso2 = g2o::Isometry2::Identity();
    iso2.translate(t);
    iso2.rotate(Eigen::Rotation2Dd(est[IDX2_YAW]));
    return iso2;
}

g2o::VectorN<PV2_STATE_SIZE> VertexPV2Radar::estimateVec() const {
    return _estimate;
}

void VertexPV2Radar::setEstimate(const g2o::Isometry2& pose) {
    // _estimate.setZero();
    _estimate[IDX2_X]      = pose.translation().x();
    _estimate[IDX2_Y]      = pose.translation().y();
    _estimate[IDX2_YAW]    = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

void VertexPV2Radar::setEstimate(const g2o::VectorN<PV2_STATE_SIZE>& pose) {
    _estimate = pose;
}
