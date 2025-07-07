/**
 * @file edge_PV2_DICP.cpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for DICP
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#include "graph_optimizer/edge_PV2_DICP.hpp"

// point to camera projection, monocular
EdgePV2DICP::EdgePV2DICP():
    BaseUnaryEdge<3, g2o::Isometry2, VertexPV2Radar>() {
    setMeasurement(g2o::Isometry2::Identity());
    information().setIdentity();
}

void EdgePV2DICP::computeError() {
    VertexPV2Radar* from  = static_cast<VertexPV2Radar*>(_vertices[0]);
    g2o::Isometry2 X     = from->estimateIso2();
    g2o::SE2 delta(_inverseMeasurement * X);
    _error = delta.toVector();

    // 정보행렬 변환 (local → global)
    Eigen::Matrix2d R = X.rotation();
    // 3x3 block-diagonal 변환행렬 T 생성
    Eigen::Matrix<double, 3, 3> T = Eigen::Matrix<double, 3, 3>::Zero();
    T.block<2,2>(0,0) = R;
    T(2,2) = 1.0;
        
    // 정보행렬 변환
    Eigen::Matrix<double, 3, 3> Info_global = T * _information_scaled_local * T.transpose();
    _information = Info_global;
}

void EdgePV2DICP::setMeasurement(const g2o::Isometry2& m) {
    _measurement        = m;
    _inverseMeasurement = m.inverse();
}
void EdgePV2DICP::getMeasurement(g2o::Isometry2& measurement) const {
    measurement = _measurement;
}

void EdgePV2DICP::setInformation(const g2o::MatrixN<3>& information) {
    _information_local = information;
}

void EdgePV2DICP::getInformation(std::vector<double>& information) const {
    for ( int i = 0; i < 3; ++i ) {
        for ( int j = 0; j < 3; ++j ) {
            information.push_back(_information(i, j));
        }
    }
}

Eigen::Isometry2d EdgePV2DICP::AlignCloudsLocalDoppler3DoF(std::vector<SRadarPoint>& source_global, const std::vector<SRadarPoint>& target,
                                                        Eigen::Isometry2d& last_icp_pose, const Velocity2D& sensor_velocity, double trans_th,
                                                        double vel_th, VoxelRcsMapperConfig m_config) {
    // sensor_velocity : last sensor frame based relative velocity

    Eigen::Matrix3d JTJ = Eigen::Matrix3d::Zero(); // 3x3 J^T J
    Eigen::Vector3d JTr = Eigen::Vector3d::Zero(); // 3x1 J^T R

    Eigen::Matrix2d mahalanobis_global = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d mahalanobis_local = Eigen::Matrix2d::Identity();
    Eigen::Matrix3d mahalanobis_local_3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix2d sensor_rot = last_icp_pose.rotation();

    Eigen::Isometry2d last_icp_pose_inv = last_icp_pose.inverse();
    Eigen::Matrix2d sensor_rot_inv = sensor_rot.inverse();

    Eigen::Vector3d point_direction_vector_sensor = Eigen::Vector3d::Zero();

    double target_static_weight = 1.0;

    Eigen::Vector2d est_sensor_vel_sensor_coord = sensor_velocity.linear;
    for (size_t i = 0; i < source_global.size(); ++i) {
        Eigen::Vector3d target_hom(target[i].pose.x(), target[i].pose.y(), 1.0);
        Eigen::Vector3d target_hom_local = last_icp_pose_inv * target_hom;
        
        const Eigen::Vector2d target_local = target_hom_local.head<2>();
        const Eigen::Vector2d residual_local = target_local - source_global[i].local.head<2>();

        Eigen::Matrix2_3d J_g; // Jacobian Geometry (2x3)
        Eigen::Matrix1_3d J_v; // Jacobian Velocity (1x3)
        Eigen::Matrix3_3d J_tot; // Jacobain Total (3x3)
        Eigen::Matrix1_3d R_tot; // Residual Total (1x3)
        
        target_static_weight = 1.0;
        
        if(target[i].is_static == false){
            target_static_weight = 0.1;
        }

        mahalanobis_global = Eigen::Matrix2d::Identity();
        mahalanobis_local = Eigen::Matrix2d::Identity();
        mahalanobis_local_3 = Eigen::Matrix3d::Identity();


        if(m_config.icp_method == IcpMethod::P2VoxelCov || m_config.icp_method == IcpMethod::P2AllVoxelCov){

            Eigen::Matrix2d RCR;
            const Eigen::Matrix2d cov_A = source_global[i].cov.block<2,2>(0,0); // source_global cov is sensor local based
            const Eigen::Matrix2d cov_B = target[i].cov.block<2,2>(0,0); // target cov is global map based

            RCR = cov_B + last_icp_pose.matrix().block<2,2>(0,0) * cov_A * last_icp_pose.matrix().block<2,2>(0,0).transpose(); // global map based covariance

            if (RCR.determinant() != 0) {
                mahalanobis_global = RCR.inverse().block<2, 2>(0, 0);
            } else {
                continue;
            }
            mahalanobis_local = sensor_rot_inv.transpose() * mahalanobis_global * sensor_rot_inv;
            mahalanobis_local_3.block<2,2>(0,0) = mahalanobis_local;

            Eigen::Vector3d error_global = Eigen::Vector3d::Zero();
            error_global = (target[i].pose - source_global[i].pose); // error in global frame
        }

        // 1. Jacobian Geometry
        // [ I(2x2) , -(R p_k)^ ] (2x3)
        J_g << 1.0, 0.0, -source_global[i].local.y(),
               0.0, 1.0,  source_global[i].local.x();

        double weight_g = square(trans_th) / (square(trans_th) + square(residual_local.squaredNorm())) * target_static_weight; 
        if(std::isnan(weight_g)){
            continue;
        }

        if(m_config.icp_use_doppler == false){
            // 6x6 += scala * (3x6).t * (3x3) * (3x6) 
            JTJ.noalias() += weight_g * J_g.transpose() * mahalanobis_local * J_g;
            JTr.noalias() += weight_g * J_g.transpose() * mahalanobis_local * residual_local; 

            // static 속성 부여
            if( residual_local.squaredNorm() < m_config.static_residual_geo * m_config.static_residual_geo){
                source_global[i].is_static = true;
            }
            else{
                source_global[i].is_static = false;
            }
            continue; 
        }

        // 2. Jacobian Velocity
        // V_D_k - V_est(T)
        const double p_azim_rad = source_global[i].azi_angle * M_PI / 180.0f; // 0 front, counterclockwise
        point_direction_vector_sensor << cos(p_azim_rad),
                                        sin(p_azim_rad), 
                                        0.0;
        // Eigen::Vector3d point_direction_vector_ego = ego_to_radar_rot_mat * point_direction_vector_sensor;

        // estimate doppler velocity based on vehicle velocity vector and observation angle
        // v_d = -d_p * V_R
        double est_point_vel = - point_direction_vector_sensor.head<2>().dot(est_sensor_vel_sensor_coord);

        // r_v = v_meas - v_d , difference between measured doppler velocity and estimated static doppler velocity
        double vel_residual = source_global[i].vel - est_point_vel;

        // [ - d_k / dt , -d_k x t_s / dt ]
        // - d_p^T / dt
        J_v = -point_direction_vector_sensor.transpose() / sensor_velocity.time_diff_sec;

        //  ( d_p X T_VR )^T / dt
        // NOTE: In 2-DoF SE2 case rotation cannot be updated by Doppler, so the cross-term is zero and
        // is therefore omitted. The previous block assignment overran matrix bounds and has been removed.

        double weight_v = square(vel_th) / (square(vel_th) + square(vel_residual)) * target_static_weight;
        if(std::isnan(weight_v)){
            continue;
        }

        // static 속성 부여
        if(fabs(vel_residual) < m_config.static_residual_vel && residual_local.squaredNorm() < m_config.static_residual_geo * m_config.static_residual_geo){
            source_global[i].is_static = true;
        }
        else{
            source_global[i].is_static = false;
        }

        double sqrt_trans_weight = sqrt( weight_g * (1.0 - m_config.doppler_trans_lambda));
        double sqrt_vel_weight   = sqrt( weight_v * m_config.doppler_trans_lambda );

        J_tot.block<2,3>(0,0) = J_g * sqrt_trans_weight; // Geometric Jacobian
        J_tot.row(2)          = J_v * sqrt_vel_weight;   // Doppler Jacobian

        // Geometric & Doppler residual (row-vector 1×3)
        R_tot << sqrt_trans_weight * residual_local.x(),
                 sqrt_trans_weight * residual_local.y(),
                 sqrt_vel_weight   * vel_residual;

        // 3x3 += (3x3).t * (3x3) * (3x3) 
        JTJ.noalias() += J_tot.transpose() * mahalanobis_local_3 * J_tot; // 3x3 = 3x3 3x3
        JTr.noalias() += J_tot.transpose() * mahalanobis_local_3 * R_tot.transpose(); // 3x1 = 3x3 3x1
    }

    // const Eigen::Vector6d x_tot = JTJ.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(JTr);
    const Eigen::Vector3d x_tot = JTJ.ldlt().solve(JTr);

    const double dtheta = x_tot(2);
    Eigen::Isometry2d transformation = Eigen::Isometry2d::Identity();
    transformation.rotate(Eigen::Rotation2Dd(dtheta));
    transformation.pretranslate(x_tot.head<2>());

    // 포인트 수에 따라 최종 정보 행렬 조정
    if (source_global.size() > 0) {
        // 포인트 수의 제곱근에 비례하게 정보 행렬 스케일링
        double point_count_scale = std::sqrt(static_cast<double>(source_global.size())) / 200.0;
        point_count_scale = std::min(2.0, std::max(0.001, point_count_scale)); // 범위 제한
        
        // 정적 포인트 비율에 따른 스케일링
        int static_point_count = 0;
        for (const auto& point : source_global) {
            if (point.is_static) static_point_count++;
        }
        double static_ratio = static_point_count / static_cast<double>(source_global.size());
        double static_scale = 0.2 + 0.8 * static_ratio; // 정적 포인트가 많을수록 더 큰 가중치
        
        // 최종 정보 행렬 계산
        double scale = (point_count_scale * static_scale);

        // 기존 정보행렬과 융합 (스케일링)
        _information_scaled_local = _information_local * scale;
        // std::cout << "information_matrix: \n" << _information << std::endl;
    }

    // Source Point based sensor local transformation
    return Eigen::Isometry2d(transformation);
}

Eigen::Isometry2d EdgePV2DICP::RunRegister(const std::vector<SRadarPoint>& source_local, std::vector<SRadarPoint>& o_frame,
                                        const Voxel2DHashMap& voxel_map, const Eigen::Isometry2d& initial_guess,
                                        const Eigen::Isometry2d& last_pose, const double dt, double trans_sigma, double vel_sigma,
                                        VoxelRcsMapperConfig m_config) {
    o_frame                       = source_local;
    m_config.doppler_trans_lambda = std::min(std::max(m_config.doppler_trans_lambda, 0.0), 1.0);
    std::set<int>                    source_set; // index of associated points in source_global
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    std::vector<SRadarPoint>         source_c_global, target_c_global;

    // 1. Create source in global frame and calculate point uncertainty
    std::vector<SRadarPoint> source_global;
    source_global.resize(source_local.size());
    TransformPoints(initial_guess, source_local, source_global);

    if ( voxel_map.Empty() ) {
        return initial_guess;
    }

    // 2. Perform ICP
    Eigen::Isometry2d last_icp_pose    = initial_guess;
    Eigen::Isometry2d estimation_local = Eigen::Isometry2d::Identity(); // ICP small change in sensor frame
    Velocity2D        iter_velocity;

    auto   start                        = std::chrono::steady_clock::now();
    int    i_iteration                  = 0;
    double total_correspondence_time_ms = 0;
    for ( int j = 0; j < m_config.icp_max_iteration; ++j ) {
        i_iteration++;
        // Get Correspodence in global frame
        // source_indices : index of valid associated points in source_global
        // source_c_global : valid associated points in source_global
        // target_c_global : the point or cov in map associated with source_c_global

        auto correspondence_start = std::chrono::steady_clock::now();

        switch ( m_config.icp_method ) {
        case IcpMethod::P2P:
            if ( m_config.use_tbb ) {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesWithSetTbb(source_global, trans_sigma * 3);
                break;
            }
            else {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesWithSet(source_global, trans_sigma * 3);
                break;
            }
        case IcpMethod::P2VoxelCov:
            if ( m_config.use_tbb ) {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesVoxelCovWithSetTbb(source_global, trans_sigma * 3);
                break;
            }
            else {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesVoxelCovWithSet(source_global, trans_sigma * 3);
                break;
            }
        case IcpMethod::P2AllVoxelCov:
            if ( m_config.use_tbb ) {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesAllVoxelCovWithSetTbb(source_global, trans_sigma * 3);
                break;
            }
            else {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesAllVoxelCovWithSet(source_global, trans_sigma * 3);
                break;
            }
        default:
            if ( m_config.use_tbb ) {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesWithSetTbb(source_global, trans_sigma * 3);
                break;
            }
            else {
                std::tie(vec_cor_origin_pair, source_c_global, target_c_global) =
                        voxel_map.GetCorrespondencesWithSet(source_global, trans_sigma * 3);
                break;
            }
        }

        auto correspondence_end = std::chrono::steady_clock::now();
        auto correspondence_elapsed_ms =
                std::chrono::duration_cast<std::chrono::microseconds>(correspondence_end - correspondence_start).count();
        total_correspondence_time_ms += correspondence_elapsed_ms;

        // std::cout << "[GraphOptimizer] Total Correspondence Time for: " << i_iteration << " in " << (correspondence_elapsed_ms / 1000.0)
        //           << " ms" << std::endl;

        // std::cout<<"[RunRegister] Asso Distance: "<<trans_sigma * 3<<std::endl;

        iter_velocity    = CalculateVelocity2D(last_pose.inverse() * last_icp_pose, dt); // last pose 기준 상대속도
        estimation_local = AlignCloudsLocalDoppler3DoF(source_c_global, target_c_global, last_icp_pose, iter_velocity, trans_sigma / 3.0,
                                                       vel_sigma / 3.0, m_config);

        // calculate estimated pose in global frame
        last_icp_pose = last_icp_pose * estimation_local;

        // small change in global frame based on global pose change
        TransformPoints(last_icp_pose, source_local, source_global);

        // check ICP termination condition
        double rot_norm = atan2(estimation_local.rotation()(1, 0), estimation_local.rotation()(0, 0));

        double transform_norm = rot_norm + estimation_local.translation().head<2>().norm();
        if ( transform_norm < m_config.icp_termination_threshold_m )
            break;
    }

    auto end        = std::chrono::steady_clock::now();
    auto elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // std::cout << "[GraphOptimizer] Total Correspondence Time: " << (total_correspondence_time_ms / 1000.0) << " ms" << std::endl;
    // std::cout << "[GraphOptimizer] RunIcpAll RunRegister: iteration executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

    // std::cout << "[GraphOptimizer] RunIcpAll RunRegister: Iteration: " << i_iteration << std::endl;

    start = std::chrono::steady_clock::now();

    for ( const auto& cor_origin_idx : vec_cor_origin_pair ) {
        if ( source_c_global[cor_origin_idx.first].is_static == true ) {
            o_frame[cor_origin_idx.second].is_static = true;
        }
    }


    end        = std::chrono::steady_clock::now();
    elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[GraphOptimizer] RunIcpAll RunRegister: Static executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

    return last_icp_pose;
}