/**
 * @file octomap_voxel_builder.hpp
 * @brief Helper for creating OctoMap-ready voxel messages from Radar4VoxMap voxels
 */

#ifndef __RADAR_4_VOX_MAP_OCTOMAP_VOXEL_BUILDER_HPP__
#define __RADAR_4_VOX_MAP_OCTOMAP_VOXEL_BUILDER_HPP__

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "radar_4_vox_map.hpp"

class Radar4VoxMapOctomapVoxelBuilder {
public:
    struct GroundPlaneModel {
        bool valid{false};
        Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
        double offset{0.0};
        int inlier_count{0};
    };

    struct Config {
        bool ground_filter_enable{true};
        std::string ground_filter_method{"height"};
        double ground_covariance_sigma_scale{1.0};
        int plane_ransac_iterations{80};
        double plane_fit_distance_threshold{0.15};
        double plane_remove_above_threshold{0.15};
        double plane_max_tilt_deg{20.0};
        int plane_min_inliers{20};
        double plane_candidate_max_height{0.6};
        double plane_marker_size{12.0};
        Eigen::Isometry3d radar_extrinsic{Eigen::Isometry3d::Identity()};
    };

    struct Input {
        const std::vector<VoxelHashMap::VoxelBlock>& voxel_blocks;
        Eigen::Matrix4d radar_pose{Eigen::Matrix4d::Identity()};
        int voxel_visualize_min_point{1};
        bool voxel_visualize_only_static{false};
    };

    explicit Radar4VoxMapOctomapVoxelBuilder(const Config& config);

    GroundPlaneModel estimateGroundPlane(const Input& input) const;

    bool shouldExportVoxel(const VoxelHashMap::VoxelBlock& voxel,
                           const Input& input,
                           const GroundPlaneModel& ground_plane) const;

    Eigen::Vector3d transformGroundFilterPointToWorld(const Eigen::Vector3d& point_filter,
                                                      const Eigen::Matrix4d& radar_pose) const;

    Eigen::Vector3d transformGroundFilterVectorToWorld(const Eigen::Vector3d& vector_filter,
                                                       const Eigen::Matrix4d& radar_pose) const;

private:
    Eigen::Vector3d transformWorldPointToGroundFilterFrame(const Eigen::Vector3d& point_world,
                                                           const Eigen::Matrix4d& radar_pose) const;

    Eigen::Matrix3d transformWorldCovarianceToGroundFilterFrame(const Eigen::Matrix3d& covariance_world,
                                                                const Eigen::Matrix4d& radar_pose) const;

    bool isGroundVoxel(const VoxelHashMap::VoxelBlock& voxel,
                       const GroundPlaneModel& ground_plane,
                       const Eigen::Matrix4d& radar_pose) const;

    bool isGroundVoxelByHeight(const VoxelHashMap::VoxelBlock& voxel,
                               const Eigen::Matrix4d& radar_pose) const;

    bool passesVisualizationFilter(const VoxelHashMap::VoxelBlock& voxel,
                                   const Input& input) const;

private:
    Config config_;
};

#endif  // __RADAR_4_VOX_MAP_OCTOMAP_VOXEL_BUILDER_HPP__
