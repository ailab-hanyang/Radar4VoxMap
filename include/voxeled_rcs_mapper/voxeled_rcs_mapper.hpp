/**
 * @file voxeled_rcs_mapper.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>, Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Voxeled RCS Mapper interface
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */

#ifndef __VOXEL_RCS_MAPPER_HPP__
#define __VOXEL_RCS_MAPPER_HPP__

#include <cmath>  
#include <numeric>
#include <cstddef>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <tuple>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "utils.hpp"

struct VoxelHashMap{

    using RadarPointVector = std::vector<SRadarPoint>;
    using RadarPointVectorTuple = std::tuple<RadarPointVector, RadarPointVector>;
    using Voxel = Eigen::Vector3i;

    struct VoxelBlock {
        // buffer of points with a max limit of n_points
        std::vector<SRadarPoint> points; // number of points
        SCovariance covariance;
        Eigen::Vector3d voxel_center;
        size_t hash_value;
        double ref_rcs;
        bool is_static;
        int max_num_points;

        bool is_changed{false};
        

        // default constructor
        VoxelBlock()
            : points(),  // initialize with empty vector
            covariance(), // initialize with default constructor of SCovariance
            voxel_center(Eigen::Vector3d::Zero()), // initialize with 0 vector
            hash_value(0),
            ref_rcs(0.0), // initialize with 1 e^0
            is_static(false),
            max_num_points(0) // initialize with 0
        {}
        VoxelBlock(const std::vector<SRadarPoint>& points,
                const SCovariance& covariance,
                const Eigen::Vector3d& voxel_center,
                size_t hash_value,
                double ref_rcs,
                bool is_static,
                int max_num_points)
            : points(points),
            covariance(covariance),
            voxel_center(voxel_center),
            hash_value(hash_value),
            ref_rcs(ref_rcs),
            is_static(is_static),
            max_num_points(max_num_points)
        {
            // this->points.reserve(max_num_points);
        }

        inline void AddPoint(const SRadarPoint &point) {
            if (points.size() < static_cast<size_t>(max_num_points)){
                points.push_back(point);
            }
        }
        inline bool AddPointValid(const SRadarPoint &point) {
            if (points.size() < static_cast<size_t>(max_num_points)){
                points.push_back(point);
                return true;
            }
            else{
                return false;
            }
        }
        inline void Reset() {
            points.clear();  // clear points vector
            covariance.cov = Eigen::Matrix3d::Identity();  // initialize with identity matrix
            covariance.mean = Eigen::Vector3d::Zero();  // initialize with 0 vector
            ref_rcs = 0.0;  // initialize with 0
            is_static = false;  // initialize with false
        }
        inline void CalCov(const bool use_tbb, const bool use_rcs_weight, const bool use_range_weight) {
            int n = points.size();

            covariance.cov = Eigen::Matrix3d::Identity();
            covariance.mean = Eigen::Vector3d::Zero();

            if(n == 0){
                return;
            }
            else if(n == 1){
                covariance.mean = points[0].pose;
                ref_rcs = points[0].rcs;
                is_static = points[0].is_static;
                return;
            }

            // Row: 3(x,y,z), Column: number of points
            Eigen::Matrix<double, 3, -1> neighbors(3, n);
            Eigen::VectorXd weights(n);  // weight vector

            double norm_rcs;
            double weight_sum = 0.0;
            int point_static_count = 0;

            // calculate max and min rcs
            double max_rcs = -1e5;
            double min_rcs = 1e5;

            for (int j = 0; j < n; j++) {
                if(points[j].rcs > max_rcs) max_rcs = points[j].rcs;
                if(points[j].rcs < min_rcs) min_rcs = points[j].rcs;
            }

            // 0 ~ 1 to 0.25 ~ 1
            max_rcs = max_rcs;
            min_rcs = max_rcs - (max_rcs - min_rcs) * 4.0 / 3.0;

            // calculate weight while calculating max and min rcs
            for (int j = 0; j < n; j++) {
                norm_rcs = SmoothStep(points[j].rcs, min_rcs, max_rcs);
                neighbors.col(j) = points[j].pose;

                // use normalized rcs as weight
                weights(j) = use_rcs_weight ? norm_rcs : 1.0;
                if(use_range_weight == true){
                    // use inverse of range as weight
                    weights(j) *= 20.0 / std::max(20.0, points[j].range);
                }

                if(points[j].is_static == true){
                    point_static_count++; // static
                }
                else{
                    weights(j) *= 0.1; // dynamic
                }

                weight_sum += weights(j);

                if(points[j].rcs > max_rcs)
                    max_rcs = points[j].rcs;
            }
        
            // if more than half of the points are static, then the voxel is static
            is_static = (static_cast<double>(point_static_count) / n > 0.5);

            // if weight is 0, then covariance calculation is not possible
            if (weight_sum == 0.0) {
                return;
            }

            // calculate weighted mean for each row (axis)
            Eigen::Vector3d weighted_mean = (neighbors * weights).rowwise().sum() / weight_sum;
            
            // subtract mean from each column and calculate covariance
            neighbors.colwise() -= weighted_mean;
            covariance.cov = (neighbors * weights.asDiagonal() * neighbors.transpose()) / weight_sum;
            
            covariance.mean = weighted_mean;

            ref_rcs = max_rcs; // TODO: currently, representative rcs is the max rcs in the voxel
        }

    };
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    size_t VoxelToSizeT(const Voxel &voxel) {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return static_cast<size_t>(vec[0]) * 73856093 ^ static_cast<size_t>(vec[1]) * 19349669 ^ static_cast<size_t>(vec[2]) * 83492791;
    }

    VoxelHashMap(){
    }
    VoxelHashMap(VoxelRcsMapperConfig config){
        m_config = config;

        int max_num_threads_ = m_config.max_thread_num > 0 ? m_config.max_thread_num
                                           : tbb::this_task_arena::max_concurrency(); 
        // This global variable requires static duration storage to be able to manipulate the max
        // concurrency from TBB across the entire class
        static const auto tbb_control_settings = tbb::global_control(
            tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
                
    }
    void Init(VoxelRcsMapperConfig config);

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesWithSet(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesWithSetTbb(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesVoxelCovWithSet(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesVoxelCovWithSetTbb(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesAllVoxelCovWithSet(const RadarPointVector &points,
                                             double max_correspondance_distance) const;

    std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
                            GetCorrespondencesAllVoxelCovWithSetTbb(const RadarPointVector &points,
                                             double max_correspondance_distance) const;


    // inline void Clear() { m_voxel_map.clear(); m_map_frame_point_count.clear();}
    inline void Clear() 
    {
        for (auto& each_voxel : m_voxel_map) {
            each_voxel.second.Reset();  // reset each VoxelBlock
        }
    }

    inline bool Empty() const { return m_voxel_map.empty(); }
    void Update(const RadarPointVector &points, const Eigen::Vector3d &origin);
    void Update(const RadarPointVector &points, const Eigen::Matrix4d &pose);
    void UpdateGlobal(const RadarPointVector &points, const Eigen::Matrix4d &pose);
    void AddPoints(const RadarPointVector &points);
    // void AddPointsWithOverlapDeletion(const RadarPointVector &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    // void RemovePointsFarfromTime(const double cur_timestamp);
    void RemovePointsFrameIdx(const int idx);
    void UpdateCovWithSet(std::unordered_set<Voxel, VoxelHash> set_voxel_check);

    std::vector<Voxel> GetSearchVoxel(const SRadarPoint &point) const;

    inline Voxel PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) const {
        return VoxelHashMap::Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
                            static_cast<int>(std::floor(point.y() / voxel_size)),
                            static_cast<int>(std::floor(point.z() / voxel_size)));
    }
    inline void PointToVoxel(const Eigen::Vector3d &point, VoxelHashMap::Voxel & o_voxel, const double voxel_size) const {
        o_voxel.x() = static_cast<int>(std::floor(point.x() / voxel_size));
        o_voxel.y() = static_cast<int>(std::floor(point.y() / voxel_size));
        o_voxel.z() = static_cast<int>(std::floor(point.z() / voxel_size));
    }
    inline std::vector<SRadarPoint> VoxelDownsample(const std::vector<SRadarPoint> &points, const double voxel_size) {
        // Voxel as key, first SRadarPoint as value
        std::unordered_map<VoxelHashMap::Voxel, SRadarPoint, VoxelHashMap::VoxelHash> grid;
        grid.reserve(points.size());

        // calculate voxel coordinate based on SRadarPoint's pose, and add to grid if voxel does not exist
        std::for_each(points.cbegin(), points.cend(), [&](const SRadarPoint &point) {
            VoxelHashMap::Voxel voxel = VoxelHashMap::PointToVoxel(point.pose, voxel_size);
            if (grid.find(voxel) == grid.end()) {
                grid.insert({voxel, point});
            }
        });

        // vector to store downsampled SRadarPoint
        std::vector<SRadarPoint> points_downsampled;
        points_downsampled.reserve(grid.size());

        // store SRadarPoint in hashmap to downsampled result
        std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
            points_downsampled.emplace_back(voxel_and_point.second);
        });

        return points_downsampled;
    }

    std::vector<SRadarPoint> Pointcloud() const;
    std::vector<SRadarPoint> StaticPointcloud() const;
    std::vector<VoxelHashMap::VoxelBlock> GetAllVoxel() const;

    std::unordered_map<Voxel, VoxelBlock, VoxelHash> m_voxel_map; 
    std::unordered_map<int, int> m_map_frame_point_count; // TODO: check how many frame points are stored in the map for each frame idx.
    // TODO: if deletion operation occurs in the map, this information must be updated.

    VoxelRcsMapperConfig m_config;

};



class VoxeledRCSMapper {
public:
    VoxeledRCSMapper() {};
    ~VoxeledRCSMapper() {};
};

#endif // __VOXEL_RCS_MAPPER_HPP__