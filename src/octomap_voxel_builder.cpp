/**
 * @file octomap_voxel_builder.cpp
 * @brief Helper implementation for OctoMap-ready voxel message generation
 */

#include "octomap_voxel_builder.hpp"

#include <algorithm>
#include <cmath>
#include <random>

Radar4VoxMapOctomapVoxelBuilder::Radar4VoxMapOctomapVoxelBuilder(const Config& config)
    : config_(config) {}

Radar4VoxMapOctomapVoxelBuilder::GroundPlaneModel
Radar4VoxMapOctomapVoxelBuilder::estimateGroundPlane(const Input& input) const {
    GroundPlaneModel model;

    if (!config_.ground_filter_enable || config_.ground_filter_method != "plane") {
        return model;
    }

    std::vector<Eigen::Vector3d> candidates;
    candidates.reserve(input.voxel_blocks.size());
    for (const auto& voxel : input.voxel_blocks) {
        if (!passesVisualizationFilter(voxel, input)) {
            continue;
        }

        const Eigen::Vector3d point = transformWorldPointToGroundFilterFrame(voxel.covariance.mean,
                                                                             input.radar_pose);
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
            continue;
        }
        if (std::abs(point.z()) > config_.plane_candidate_max_height) {
            continue;
        }
        candidates.push_back(point);
    }

    if (candidates.size() < static_cast<size_t>(std::max(3, config_.plane_min_inliers))) {
        return model;
    }

    const double max_tilt_rad = config_.plane_max_tilt_deg * M_PI / 180.0;
    const double min_normal_z = std::cos(max_tilt_rad);

    std::mt19937 rng(17);
    std::uniform_int_distribution<size_t> index_dist(0, candidates.size() - 1);

    std::vector<int> best_inlier_indices;
    best_inlier_indices.reserve(candidates.size());
    Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
    double best_offset = 0.0;

    for (int iteration = 0; iteration < config_.plane_ransac_iterations; ++iteration) {
        size_t idx_a = index_dist(rng);
        size_t idx_b = index_dist(rng);
        size_t idx_c = index_dist(rng);
        if (idx_a == idx_b || idx_a == idx_c || idx_b == idx_c) {
            continue;
        }

        const Eigen::Vector3d& a = candidates[idx_a];
        const Eigen::Vector3d& b = candidates[idx_b];
        const Eigen::Vector3d& c = candidates[idx_c];

        Eigen::Vector3d normal = (b - a).cross(c - a);
        const double normal_norm = normal.norm();
        if (normal_norm < 1e-6) {
            continue;
        }
        normal /= normal_norm;
        if (normal.z() < 0.0) {
            normal = -normal;
        }
        if (normal.z() < min_normal_z) {
            continue;
        }

        const double offset = -normal.dot(a);
        std::vector<int> inlier_indices;
        inlier_indices.reserve(candidates.size());
        for (size_t i = 0; i < candidates.size(); ++i) {
            const double distance = std::abs(normal.dot(candidates[i]) + offset);
            if (distance < config_.plane_fit_distance_threshold) {
                inlier_indices.push_back(static_cast<int>(i));
            }
        }

        if (inlier_indices.size() > best_inlier_indices.size()) {
            best_inlier_indices = std::move(inlier_indices);
            best_normal = normal;
            best_offset = offset;
        }
    }

    if (best_inlier_indices.size() < static_cast<size_t>(config_.plane_min_inliers)) {
        return model;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const int index : best_inlier_indices) {
        centroid += candidates[index];
    }
    centroid /= static_cast<double>(best_inlier_indices.size());

    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const int index : best_inlier_indices) {
        const Eigen::Vector3d centered = candidates[index] - centroid;
        scatter.noalias() += centered * centered.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(scatter);
    if (eigensolver.info() == Eigen::Success) {
        best_normal = eigensolver.eigenvectors().col(0);
        if (best_normal.z() < 0.0) {
            best_normal = -best_normal;
        }
        best_offset = -best_normal.dot(centroid);
    }

    if (best_normal.z() < min_normal_z) {
        return model;
    }

    model.valid = true;
    model.normal = best_normal;
    model.offset = best_offset;
    model.inlier_count = static_cast<int>(best_inlier_indices.size());
    return model;
}

bool Radar4VoxMapOctomapVoxelBuilder::shouldExportVoxel(const VoxelHashMap::VoxelBlock& voxel,
                                                        const Input& input,
                                                        const GroundPlaneModel& ground_plane) const {
    return passesVisualizationFilter(voxel, input) &&
           !isGroundVoxel(voxel, ground_plane, input.radar_pose);
}

Eigen::Vector3d Radar4VoxMapOctomapVoxelBuilder::transformWorldPointToGroundFilterFrame(const Eigen::Vector3d& point_world,
                                                                                        const Eigen::Matrix4d& radar_pose) const {
    Eigen::Vector4d point_world_h;
    point_world_h << point_world, 1.0;
    const Eigen::Vector4d point_in_current_radar_h = radar_pose.inverse() * point_world_h;
    const Eigen::Vector3d point_in_current_radar = point_in_current_radar_h.head<3>();
    return config_.radar_extrinsic * point_in_current_radar;
}

Eigen::Matrix3d Radar4VoxMapOctomapVoxelBuilder::transformWorldCovarianceToGroundFilterFrame(const Eigen::Matrix3d& covariance_world,
                                                                                             const Eigen::Matrix4d& radar_pose) const {
    const Eigen::Matrix3d world_to_current_radar_rotation = radar_pose.block<3, 3>(0, 0).transpose();
    const Eigen::Matrix3d world_to_ground_filter_rotation = config_.radar_extrinsic.linear() *
                                                            world_to_current_radar_rotation;
    return world_to_ground_filter_rotation * covariance_world * world_to_ground_filter_rotation.transpose();
}

Eigen::Vector3d Radar4VoxMapOctomapVoxelBuilder::transformGroundFilterPointToWorld(const Eigen::Vector3d& point_filter,
                                                                                   const Eigen::Matrix4d& radar_pose) const {
    const Eigen::Vector3d point_radar = config_.radar_extrinsic.inverse() * point_filter;
    Eigen::Vector4d point_radar_h;
    point_radar_h << point_radar, 1.0;
    return (radar_pose * point_radar_h).head<3>();
}

Eigen::Vector3d Radar4VoxMapOctomapVoxelBuilder::transformGroundFilterVectorToWorld(const Eigen::Vector3d& vector_filter,
                                                                                    const Eigen::Matrix4d& radar_pose) const {
    const Eigen::Vector3d vector_radar = config_.radar_extrinsic.linear().transpose() *
                                         vector_filter;
    return radar_pose.block<3, 3>(0, 0) * vector_radar;
}

bool Radar4VoxMapOctomapVoxelBuilder::isGroundVoxel(const VoxelHashMap::VoxelBlock& voxel,
                                                    const GroundPlaneModel& ground_plane,
                                                    const Eigen::Matrix4d& radar_pose) const {
    if (!config_.ground_filter_enable) {
        return false;
    }

    if (config_.ground_filter_method != "plane" || !ground_plane.valid) {
        return isGroundVoxelByHeight(voxel, radar_pose);
    }

    const Eigen::Vector3d mean_in_ground_filter_frame = transformWorldPointToGroundFilterFrame(voxel.covariance.mean,
                                                                                               radar_pose);
    const Eigen::Matrix3d covariance_in_ground_filter_frame = transformWorldCovarianceToGroundFilterFrame(voxel.covariance.cov,
                                                                                                          radar_pose);

    const double signed_distance_to_plane = ground_plane.normal.dot(mean_in_ground_filter_frame) +
                                            ground_plane.offset;
    double variance_along_plane_normal = (ground_plane.normal.transpose() *
                                          covariance_in_ground_filter_frame *
                                          ground_plane.normal)(0, 0);
    if (!std::isfinite(variance_along_plane_normal)) {
        variance_along_plane_normal = 0.0;
    }
    const double sigma_along_plane_normal = std::sqrt(std::max(0.0, variance_along_plane_normal));

    return signed_distance_to_plane <
           config_.plane_remove_above_threshold +
               config_.ground_covariance_sigma_scale * sigma_along_plane_normal;
}

bool Radar4VoxMapOctomapVoxelBuilder::isGroundVoxelByHeight(const VoxelHashMap::VoxelBlock& voxel,
                                                            const Eigen::Matrix4d& radar_pose) const {
    if (!config_.ground_filter_enable) {
        return false;
    }

    const Eigen::Vector3d mean_in_ground_filter_frame = transformWorldPointToGroundFilterFrame(voxel.covariance.mean,
                                                                                               radar_pose);
    const Eigen::Matrix3d covariance_in_ground_filter_frame = transformWorldCovarianceToGroundFilterFrame(voxel.covariance.cov,
                                                                                                          radar_pose);

    double z_variance_in_ground_filter_frame = covariance_in_ground_filter_frame(2, 2);
    if (!std::isfinite(z_variance_in_ground_filter_frame)) {
        z_variance_in_ground_filter_frame = 0.0;
    }

    const double z_sigma_in_ground_filter_frame = std::sqrt(std::max(0.0, z_variance_in_ground_filter_frame));
    const double ground_test_z = mean_in_ground_filter_frame.z() -
                                 config_.ground_covariance_sigma_scale * z_sigma_in_ground_filter_frame;

    return ground_test_z < 0.0;
}

bool Radar4VoxMapOctomapVoxelBuilder::passesVisualizationFilter(const VoxelHashMap::VoxelBlock& voxel,
                                                                const Input& input) const {
    return voxel.points.size() >= static_cast<size_t>(input.voxel_visualize_min_point) &&
           (voxel.is_static || !input.voxel_visualize_only_static);
}
