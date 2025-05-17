/*
 * @copyright Automotive Intelligence Lab, Hanyang University
 * @author Jaeyoung Jo (wodud3743@gmail.com)
 * @file utils.hpp
 * @brief util functions for radar 4 vox map algorithm
 * @version 1.0
 * @date 2024-09-02
 */

#ifndef __UTILS_HPP__
#define __UTILS_HPP__
#pragma once

#include <numeric>
#include <vector>
#include <deque>
#include <string>
#include <map>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud2_iterator.h"
#include <regex>

#include "types/radar_point_cloud.hpp"

// g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

// g2o
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"

#include <omp.h>

#include <tbb/global_control.h>
#include <tbb/task_group.h>
#include <tbb/task_arena.h>
#include <tbb/parallel_reduce.h>   // for TBB parallel_reduce
#include <tbb/blocked_range.h>     // for TBB blocked_range
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/combinable.h>

namespace Eigen {
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    using Matrix3_3d = Eigen::Matrix<double, 3, 3>;
    using Matrix2_3d = Eigen::Matrix<double, 2, 3>;
    using Matrix1_3d = Eigen::Matrix<double, 1, 3>;
    using Matrix1_4d = Eigen::Matrix<double, 1, 4>;
    using Matrix1_6d = Eigen::Matrix<double, 1, 6>;
    using Matrix4_6d = Eigen::Matrix<double, 4, 6>;

    // using Matrix3d = Eigen::Matrix<double, 3, 1>;
}  // namespace Eigen

typedef enum {
	P2P,
    P2VoxelCov,
    P2AllVoxelCov
} IcpMethod;

struct VoxelRcsMapperConfig{
    bool use_tbb;
    int max_thread_num;

    double radar_range_variance_m;
    double radar_azimuth_variance_deg;
    double radar_elevation_variance_deg;

    int optimization_num;
    bool run_optimization;
    bool icp_all_node;
    bool use_doppler_unary_edge;
    int optimization_algorithm_type;
    double node_erase_distance;
    int vertex_num_max;
    int optimization_vertex_num_max;
    double fixed_vertex_to_map_keyframe_distance;

    std::vector<double> edge_unary_dicp_std;
    double edge_unary_doppler_information_scale;
    std::vector<double> edge_binary_cv_std;


    IcpMethod icp_method;
    int icp_max_iteration;
    double icp_termination_threshold_m;
    double doppler_trans_lambda;
    double icp_use_doppler;
    double static_residual_geo;
    double static_residual_vel;
    double scan_voxel_size;

    double voxel_size;
    double voxel_map_max_distance;
    int voxel_max_point_num;
    int voxel_search_level;
    bool voxel_map_use_rcs;
    bool voxel_map_use_range_weight;

    double initial_trans_threshold;
    double initial_vel_threshold;
    double min_motion_threshold;

    int local_map_property;
    int voxel_visualize_min_point;
    bool voxel_visualize_only_static;

    bool virtual_gravity_align;
    double virtual_gravity_align_information;
};


struct SRadarPoint {
    Eigen::Vector3d pose;
    Eigen::Matrix4d cov;

    Eigen::Vector3d local; // for doppler vel prediction
    // Eigen::Matrix4d sensor_pose;

    double power;
    double rcs;
    double range; // m
    double vel; // mps
    double azi_angle; // deg Right to Left
    double ele_angle; // deg. Down to Up

    int frame_idx;
    double timestamp;

    bool is_static;

    // 생성자
    SRadarPoint()
        : pose(Eigen::Vector3d::Zero()), cov(Eigen::Matrix4d::Identity()),
          local(Eigen::Vector3d::Zero()),
        //   sensor_pose(Eigen::Matrix4d::Identity()), 
          power(0.0), rcs(0.0),
          range(0.0), vel(0.0), azi_angle(0.0), ele_angle(0.0),
          frame_idx(0), timestamp(0.0), is_static(false){}

    ~SRadarPoint(){
    };

    // reset 함수
    void reset() {
        pose.setZero();
        cov.setZero();
        local.setZero();
        // sensor_pose.setZero();
        power = 0.0;
        rcs = 0.0;
        range = 0.0;
        vel = 0.0;
        azi_angle = 0.0;
        ele_angle = 0.0;
        frame_idx = 0;
        timestamp = 0.0;
        is_static = false;
    }
};

struct Velocity {
    Eigen::Vector3d linear;  // linear velocity
    Eigen::Vector3d angular; // angular velocity

    double time_diff_sec;

    // default constructor - initialize member variables to 0
    Velocity()
        : linear(Eigen::Vector3d::Zero()), angular(Eigen::Vector3d::Zero()), time_diff_sec(0.1) {}

    // constructor with parameters
    Velocity(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, const double& time_diff)
        : linear(lin), angular(ang), time_diff_sec(time_diff) {}
};

struct SCovariance {
    Eigen::Matrix3d cov;  // 3x3 covariance matrix
    Eigen::Vector3d mean; // 3D mean vector

    SCovariance() : cov(Eigen::Matrix3d::Identity()), mean(Eigen::Vector3d::Zero()) {}

    SCovariance(const Eigen::Matrix3d& c, const Eigen::Vector3d& m) 
        : cov(c), mean(m) {}
};

// Visualization of graph
struct SVisVertex{
    Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();

    int id{0};
};

struct SVisEdgeBinary{
    Eigen::Matrix4d vertex_pose_start{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d vertex_pose_end{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d measurement{Eigen::Matrix4d::Identity()};
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();

    int id{0};
    int id_start{0};
    int id_end{0};
};

struct SVisEdgeUnary{
    Eigen::Matrix4d vertex_pose{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d measurement{Eigen::Matrix4d::Identity()};
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();

    int id{0};
    int id_start{0};
};

struct SVisEdgeUnaryDoppler{
    Eigen::Matrix4d vertex_pose{Eigen::Matrix4d::Identity()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Zero();

    int id{0};
    int id_start{0};
};

struct AdaptiveThreshold {

    void Init(VoxelRcsMapperConfig config){
        initial_trans_threshold_ = config.initial_trans_threshold;
        initial_vel_threshold_ = config.initial_vel_threshold;
        min_motion_th_ = config.min_motion_threshold;
        max_range_ = config.voxel_map_max_distance;
    }

    /// Update the current belief of the deviation from the prediction model
    inline void UpdateModelDeviation(const Eigen::Matrix4d &current_deviation) {
        model_deviation_ = current_deviation;
    }

    double ComputeTransModelError(const Eigen::Matrix4d &model_deviation, double max_range) {
        const float theta = Eigen::AngleAxisd(model_deviation.block<3,3>(0,0)).angle();
        const float delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
        const float delta_trans = model_deviation.block<3,1>(0,3).norm();
        return double(delta_trans + delta_rot);
    }

    double ComputeVelModelError(const Eigen::Matrix4d &model_deviation, double max_range) {
        const float delta_trans = model_deviation.block<3,1>(0,3).norm();
        return double(delta_trans / 0.1);
    }

    /// Returns the KISS-ICP adaptive threshold used in registration
    double ComputeTransThreshold() {
        double model_error = ComputeTransModelError(model_deviation_, max_range_);

        std::cout<<"Model Error: "<<model_error<<std::endl;

        if (model_error > min_motion_th_) {
            deque_model_error_sse2_.push_back(model_error * model_error);
        }

        if (deque_model_error_sse2_.size() < 1) {
            return initial_trans_threshold_;
        }

        while(deque_model_error_sse2_.size() > 20){
            deque_model_error_sse2_.pop_front();
        }
        
        return std::sqrt(std::accumulate(deque_model_error_sse2_.begin(), deque_model_error_sse2_.end(), 0.0) / deque_model_error_sse2_.size());
    }

    // if model_deviation < min_motion_th_, use default threshold
    double ComputeVelThreshold() {
        double vel_error = ComputeVelModelError(model_deviation_, max_range_);

        std::cout<<"vel   Error: "<<vel_error<<std::endl;

        if (vel_error > min_motion_th_) {
            deque_vel_error_sse2_.push_back(vel_error * vel_error);
        }

        if (deque_vel_error_sse2_.size() < 1) {
            return initial_vel_threshold_;
        }

        while(deque_vel_error_sse2_.size() > 20){
            deque_vel_error_sse2_.pop_front();
        }
        
        return std::sqrt(std::accumulate(deque_vel_error_sse2_.begin(), deque_vel_error_sse2_.end(), 0.0) / deque_vel_error_sse2_.size());
    }

    // configurable parameters
    double initial_trans_threshold_;
    double initial_vel_threshold_;
    double min_motion_th_;
    double max_range_;

    // Local cache for ccomputation
    double model_error_sse2_ = 0;
    int num_samples_ = 0;
   
    std::deque<double> deque_model_error_sse2_;
    std::deque<double> deque_vel_error_sse2_;

    Eigen::Matrix4d model_deviation_ = Eigen::Matrix4d::Identity();
};


inline Velocity CalculateVelocity(const Eigen::Matrix4d& transform, double delta_t_sec) {
    // rotation matrix
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    // translation vector
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    
    // linear velocity v (divide translation by time change)
    Eigen::Vector3d linear_vel = translation / delta_t_sec;
    
    // calculate angular velocity matrix omega (use logarithm map)
    Eigen::AngleAxisd angle_axis(rotation);
    Eigen::Vector3d angular_vel = angle_axis.angle() * angle_axis.axis() / delta_t_sec;
    
    // create Velocity structure and return
    Velocity velocity(linear_vel, angular_vel, delta_t_sec);
    
    return velocity;
}

inline SRadarPoint CalPointCov(const SRadarPoint point, double range_var_m, double azim_var_deg, double ele_var_deg){
    SRadarPoint cov_point = point;
    double dist = cov_point.range;
    double s_x = range_var_m;
    double s_y = std::max(0.1, dist * sin(azim_var_deg / 180 * M_PI)); // 0.00873
    double s_z = std::max(0.1, dist * sin(ele_var_deg / 180 * M_PI)); // 0.01745
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(cov_point.ele_angle / 180 * M_PI, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(cov_point.azi_angle / 180 * M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R; // Rotation matrix
    R = yawAngle * pitchAngle;

    Eigen::Matrix3d S; // Scaling matix
    S << s_x, 0.0, 0.0,
         0.0, s_y, 0.0,
         0.0, 0.0, s_z;

    Eigen::Matrix3d cov = R * S;
    cov_point.cov = Eigen::Matrix4d::Zero();
    cov_point.cov.block<3, 3>(0, 0) = cov;

    return cov_point;
}

inline void CalPointCovVoid(SRadarPoint & point, double range_var_m, double azim_var_deg, double ele_var_deg){
    double dist = point.range;
    double s_x = range_var_m;
    double s_y = std::max(0.1, dist * sin(azim_var_deg / 180 * M_PI)); // 0.00873
    double s_z = std::max(0.1, dist * sin(ele_var_deg / 180 * M_PI)); // 0.01745
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(point.ele_angle / 180 * M_PI, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(point.azi_angle / 180 * M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R; // Rotation matrix
    R = yawAngle * pitchAngle;

    Eigen::Matrix3d S; // Scaling matix
    S << s_x, 0.0, 0.0,
         0.0, s_y, 0.0,
         0.0, 0.0, s_z;

    Eigen::Matrix3d cov = R * S;
    point.cov = Eigen::Matrix4d::Zero();
    point.cov.block<3, 3>(0, 0) = cov;

    return;
}

inline void CalPointCovVoidComputed(SRadarPoint & point, double range_var, double sin_azim, double sin_ele){
    double dist = point.range;

    double s_x = range_var;
    double s_y = std::max(0.1, dist * sin_azim); // 0.00873
    double s_z = std::max(0.1, dist * sin_ele); // 0.01745
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(point.ele_angle / 180 * M_PI, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(point.azi_angle / 180 * M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R; // Rotation matrix
    R = yawAngle * pitchAngle;

    Eigen::Matrix3d S; // Scaling matix
    S << s_x, 0.0, 0.0,
         0.0, s_y, 0.0,
         0.0, 0.0, s_z;

    Eigen::Matrix3d cov = R * S;
    point.cov = Eigen::Matrix4d::Zero();
    point.cov.block<3, 3>(0, 0) = cov;

    return;
}


inline void CalFramePointCov(std::vector<SRadarPoint> &points, double range_var_m, double azim_var_deg, double ele_var_deg){
    double sin_azim = sin(azim_var_deg / 180 * M_PI);
    double sin_ele = sin(ele_var_deg / 180 * M_PI);

    for (auto& point : points) {  // for each element, directly call
        CalPointCovVoidComputed(point, range_var_m, sin_azim, sin_ele);
    }
}

void CalFramePointCovTbb(std::vector<SRadarPoint> &points, double range_var_m, double azim_var_deg, double ele_var_deg);

inline Eigen::Matrix3d vectorToSkewSymmetricMatrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -vec.z(), vec.y(),
                      vec.z(), 0, -vec.x(),
                      -vec.y(), vec.x(), 0;
    return skew_symmetric;
}

inline void TransformPoints(const Eigen::Matrix4d &T, std::vector<SRadarPoint> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) {
                       Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
                       Eigen::Vector4d p_transformed = T * p;
                       SRadarPoint transformed_point = point; // copy all properties
                       transformed_point.pose = p_transformed.head<3>();
                       return transformed_point;
                   });
}

inline void TransformPoints(const Eigen::Matrix4d &T, const std::vector<SRadarPoint> &points, std::vector<SRadarPoint> &o_points) {
    assert(points.size() == o_points.size()); // check if points and o_points have the same size

    std::transform(points.cbegin(), points.cend(), o_points.begin(),
                   [&](const auto &point) {
                       Eigen::Vector4d p(point.pose.x(), point.pose.y(), point.pose.z(), 1.0);
                       Eigen::Vector4d p_transformed = T * p;
                       SRadarPoint transformed_point = point; // copy all properties
                       transformed_point.pose = p_transformed.head<3>();
                       return transformed_point;
                   });
}


inline double square(double x) { return x * x; }

inline double Sigmoid(const double x){
    return 1.0 / (1.0 + exp(-x));
}

inline double Sigmoid(const double x, const double max_val){
    double c = log(99)/max_val; // 99 percent in max value
    return 1.0 / (1.0 + exp(-c*x));
}

inline double SmoothStep(const double x, const double min, const double max){
    if(x <= min) return 0.0;
    else if (x <= max){
        double t = (x - min)/(max - min);
        return (3*t*t - 2*t*t*t);
    }
    return 1.0;
}

inline std::vector<SRadarPoint> CropFrame(const std::vector<SRadarPoint> &frame,
                                        double max_range,
                                        double min_range) {
    std::vector<SRadarPoint> inliers;
    std::copy_if(frame.cbegin(), frame.cend(), std::back_inserter(inliers), [&](const auto &pt) {
        const double norm = pt.pose.squaredNorm();
        return norm < max_range * max_range && norm > min_range * min_range;
    });
    return inliers;
}


inline std::vector<SRadarPoint> ConvertRadarPointsToSRadarPoints(const std::vector<RadarPoint>& radar_points, int i_frame_idx) {
    std::vector<SRadarPoint> s_radar_points;
    int i_radar_points_num = radar_points.size();
    s_radar_points.resize(i_radar_points_num);  // pre-allocate vector size

    for (size_t i = 0; i < i_radar_points_num; ++i) {
        const auto& radar_point = radar_points[i];
        auto& s_radar_point = s_radar_points[i];  // directly access index

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

        // if needed, set additional fields
        // e.g. s_radar_point.sensor_pose
    }

    return s_radar_points;
}

std::vector<SRadarPoint> ConvertRadarPointsToSRadarPointsTbb(const std::vector<RadarPoint>& radar_points, int i_frame_idx);



// convert RadarPoint vector to SRadarPoint vector
inline std::vector<SRadarPoint> ConvertRadarPointsToSRadarPoints(const std::vector<RadarPoint>& radar_points) {
    return ConvertRadarPointsToSRadarPoints(radar_points, 0);
}


inline std::vector<RadarPoint> ConvertSRadarPointsToRadarPoints(const std::vector<SRadarPoint>& s_radar_points) {
    std::vector<RadarPoint> radar_points;
    int i_radar_points_num = s_radar_points.size();
    radar_points.resize(i_radar_points_num);  // pre-allocate vector size

    for (size_t i = 0; i < i_radar_points_num; ++i) {
        const auto& s_radar_point = s_radar_points[i];
        auto& radar_point = radar_points[i];  // directly access index

        radar_point.x = static_cast<float>(s_radar_point.pose.x());
        radar_point.y = static_cast<float>(s_radar_point.pose.y());
        radar_point.z = static_cast<float>(s_radar_point.pose.z());

        radar_point.range = static_cast<float>(s_radar_point.range);
        radar_point.azi = static_cast<float>(s_radar_point.azi_angle);
        radar_point.ele = static_cast<float>(s_radar_point.ele_angle);
        radar_point.vel = static_cast<float>(s_radar_point.vel);
        radar_point.rcs = static_cast<float>(s_radar_point.rcs);
        radar_point.power = static_cast<float>(s_radar_point.power);

        // if needed, set additional fields
    }

    return radar_points;
}

std::vector<RadarPoint> ConvertSRadarPointsToRadarPointsTbb(const std::vector<SRadarPoint>& s_radar_points);

std::pair<Eigen::MatrixXd, Eigen::VectorXd> getAandB(const std::vector<SRadarPoint>& points);

Eigen::Vector3d getVelocityFromDoppler(const std::vector<SRadarPoint>& points, Eigen::MatrixXd A, Eigen::VectorXd b);

std::vector<int> getInlierIndices(const Eigen::Vector3d& estimated_velocity, const std::vector<SRadarPoint>& points, const Eigen::MatrixXd& A, const double vel_threshold);

bool estimateLocalVelocityFromPoints(
    const std::vector<SRadarPoint>& points,
    Eigen::Vector3d& estimated_velocity, // Output velocity
    Eigen::Matrix3d& velocity_covariance // Output covariance
);

#endif // __UTILS_HPP__
