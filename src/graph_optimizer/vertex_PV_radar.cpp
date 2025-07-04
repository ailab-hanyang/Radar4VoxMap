/**
 * @file vertex_PV_radar.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief VertexPVRadar interface
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/vertex_PV_radar.hpp"

VertexPVRadar::VertexPVRadar():
    g2o::BaseVertex<PV_STATE_SIZE, g2o::VectorN<PV_STATE_SIZE>>() {
    setToOriginImpl();
    updateCache();
}

void VertexPVRadar::setToOriginImpl() {
    _estimate.setZero();
}

/**
 * update the position of this vertex. The update is in the form
 * (x,y,z,qx,qy,qz) whereas (x,y,z) represents the translational update
 * and (qx,qy,qz) corresponds to the respective elements. The missing
 * element qw of the quaternion is recovred by
 * || (qw,qx,qy,qz) || == 1 => qw = sqrt(1 - || (qx,qy,qz) ||
 */
void VertexPVRadar::oplusImpl(const double* update) {
    /* Pose update */
    g2o::Vector6 update_pose; // x, y, z, qx, qy, qz
    update_pose << update[IDX_X], update[IDX_Y], update[IDX_Z], update[IDX_QX], update[IDX_QY], update[IDX_QZ];
    g2o::Isometry3 increment = g2o::internal::fromVectorMQT(update_pose);

    // // estimate to isometry3
    // g2o::Vector6 estimate_pose;
    // estimate_pose << _estimate[IDX_X], _estimate[IDX_Y], _estimate[IDX_Z], _estimate[IDX_QX], _estimate[IDX_QY], _estimate[IDX_QZ];
    // g2o::Isometry3 estimate_iso3   = g2o::internal::fromVectorMQT(estimate_pose);
    // estimate_iso3                  = estimate_iso3 * increment;
    // g2o::Vector6 estimate_pose_vec = g2o::internal::toVectorMQT(estimate_iso3);
    // _estimate[IDX_X]               = estimate_pose_vec[0];
    // _estimate[IDX_Y]               = estimate_pose_vec[1];
    // _estimate[IDX_Z]               = estimate_pose_vec[2];
    // _estimate[IDX_QX]              = estimate_pose_vec[3];
    // _estimate[IDX_QY]              = estimate_pose_vec[4];
    // _estimate[IDX_QZ]              = estimate_pose_vec[5];

    _estimate[IDX_X] += update[IDX_X];
    _estimate[IDX_Y] += update[IDX_Y];
    _estimate[IDX_Z] += update[IDX_Z];

    // Angular prediction
    double             qw = std::max(0.0, std::sqrt(1 - _estimate[IDX_QX] * _estimate[IDX_QX] - _estimate[IDX_QY] * _estimate[IDX_QY] -
                                      _estimate[IDX_QZ] * _estimate[IDX_QZ]));
    Eigen::Quaterniond q_current(qw, _estimate[IDX_QX], _estimate[IDX_QY], _estimate[IDX_QZ]);

    // Update quaternion using small angle approximation
    // For small rotations, we can approximate the quaternion update as:
    // q_new = q_current * [1, dx/2, dy/2, dz/2]
    Eigen::Vector3d update_vec(update[IDX_QX], update[IDX_QY], update[IDX_QZ]);
    double update_norm = update_vec.norm();

    Eigen::Quaterniond q_delta;
    if (update_norm > 1e-6) {
        // If rotation is large enough, use proper quaternion
        double half_angle = update_norm / 2.0;
        Eigen::Vector3d axis = update_vec / update_norm;
        q_delta = Eigen::Quaterniond(std::cos(half_angle), 
                                   axis.x() * std::sin(half_angle),
                                   axis.y() * std::sin(half_angle), 
                                   axis.z() * std::sin(half_angle));
    } else {
        // For very small rotations, use linear approximation
        q_delta = Eigen::Quaterniond(1.0, update[IDX_QX]/2.0, 
                                        update[IDX_QY]/2.0, 
                                        update[IDX_QZ]/2.0);
    }

    // Update the quaternion and normalize
    Eigen::Quaterniond q_new = q_current * q_delta;
    q_new.normalize();


    _estimate[IDX_QX] = q_new.x();
    _estimate[IDX_QY] = q_new.y();
    _estimate[IDX_QZ] = q_new.z();


    /* Velocity update */
    _estimate[IDX_VX] += update[IDX_VX];
    _estimate[IDX_VY] += update[IDX_VY];
    _estimate[IDX_VZ] += update[IDX_VZ];
    _estimate[IDX_WX] += update[IDX_WX];
    _estimate[IDX_WY] += update[IDX_WY];
    _estimate[IDX_WZ] += update[IDX_WZ];
}

bool VertexPVRadar::read(std::istream& is) {
    is >> _estimate[IDX_X] >> _estimate[IDX_Y] >> _estimate[IDX_Z] >> _estimate[IDX_VX] >> _estimate[IDX_VY] >> _estimate[IDX_VZ] >>
            _estimate[IDX_QX] >> _estimate[IDX_QY] >> _estimate[IDX_QZ] >> _estimate[IDX_WX] >> _estimate[IDX_WY] >> _estimate[IDX_WZ];
    return true;
}

bool VertexPVRadar::write(std::ostream& os) const {
    os << _estimate[IDX_X] << " " << _estimate[IDX_Y] << " " << _estimate[IDX_Z] << " " << _estimate[IDX_VX] << " " << _estimate[IDX_VY]
       << " " << _estimate[IDX_VZ] << " " << _estimate[IDX_QX] << " " << _estimate[IDX_QY] << " " << _estimate[IDX_QZ] << " "
       << _estimate[IDX_WX] << " " << _estimate[IDX_WY] << " " << _estimate[IDX_WZ];
    return os.good();
}

g2o::Isometry3 VertexPVRadar::estimateIso3() const {
    g2o::VectorN<PV_STATE_SIZE> est  = _estimate;
    g2o::Vector3                t    = {est[IDX_X], est[IDX_Y], est[IDX_Z]};
    double                      qw   = std::max(0.0, std::sqrt(1 - est[IDX_QX] * est[IDX_QX] - est[IDX_QY] * est[IDX_QY] - est[IDX_QZ] * est[IDX_QZ]));
    g2o::Quaternion             q    = {qw, est[IDX_QX], est[IDX_QY], est[IDX_QZ]};
    g2o::Isometry3              iso3 = g2o::Isometry3::Identity();
    iso3.translate(t);
    iso3.rotate(q);
    return iso3;
}

g2o::VectorN<PV_STATE_SIZE> VertexPVRadar::estimateVec() const {
    return _estimate;
}

void VertexPVRadar::setEstimate(const g2o::Isometry3& pose) {
    // _estimate.setZero();
    g2o::Vector6 pose_vec = g2o::internal::toVectorMQT(pose);
    _estimate[IDX_X]      = pose_vec[0];
    _estimate[IDX_Y]      = pose_vec[1];
    _estimate[IDX_Z]      = pose_vec[2];
    _estimate[IDX_QX]     = pose_vec[3];
    _estimate[IDX_QY]     = pose_vec[4];
    _estimate[IDX_QZ]     = pose_vec[5];
}

void VertexPVRadar::setEstimate(const g2o::VectorN<PV_STATE_SIZE>& pose) {
    _estimate = pose;
}
