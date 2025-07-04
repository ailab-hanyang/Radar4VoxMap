/**
 * @file radar_4_vox_map.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>, Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Radar 4 Vox Map algorithm core
 * @version 0.1
 * @date 2024-05-20
 *
 * @note This file contains modifications based on the original code from the GitHub repository
 * https://github.com/PRBonn/kiss-icp
 * The original code was contributed by Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill
 * and is licensed under the MIT License
 * 
 * @copyright Copyright (c) 2024
 */

#ifndef __RADAR_4_VOX_MAP_HPP__
#define __RADAR_4_VOX_MAP_HPP__

#include "graph_optimizer/graph_optimizer.hpp"
#include "types/radar_point_cloud.hpp"
#include "voxeled_rcs_mapper/voxeled_rcs_mapper.hpp"

class Radar4VoxMap {
public:
    using RadarPointVector = std::vector<SRadarPoint>;
    using RadarPointVectorTuple = std::tuple<std::vector<SRadarPoint>, std::vector<SRadarPoint>>;

// Birth
public:
    Radar4VoxMap() : m_tbb_arena(tbb::task_arena::automatic), m_tbb_group(), m_motion(std::make_pair(std::vector<double>(6, 0.0), std::vector<double>(6, 0.0))){};
    ~Radar4VoxMap() {m_tbb_arena.execute([&] { m_tbb_group.wait(); });};

    void Init(VoxelRcsMapperConfig config);

// Core
public:
    std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> RunCore(std::vector<RadarPoint> points, const double i_radar_timestamp_sec);
    std::pair<std::vector<double>, std::vector<double>> GetMotion(){return m_motion;};
    std::vector<VoxelHashMap::VoxelBlock> GetAllVoxelFromMap() const {return m_graph_optimizer.GetAllVoxelFromMap();};
    std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>,std::vector<SVisEdgeUnary>> GetAllGraphElements() const {return m_graph_optimizer.GetAllGraphElements();};
    std::vector<SVisEdgeUnaryDoppler> GetVertexDopplerVel() const {return m_graph_optimizer.GetVertexDopplerVel();};
    std::vector<SRadarPoint> LocalMap() const {return m_graph_optimizer.LocalMap();};
    std::vector<SRadarPoint> StaticLocalMap() const { return m_graph_optimizer.StaticLocalMap();};

// publish
public:

// Functions
private:
    tbb::task_arena m_tbb_arena;   // task arena for TBB
    tbb::task_group m_tbb_group;   // task group for parallel tasks

private:
    VoxelRcsMapperConfig m_config;
    std::pair<std::vector<double>, std::vector<double>> m_motion;
    GraphOptimizer m_graph_optimizer;
    // VoxelHashMap m_voxel_hash_map;

};

#endif // __RADAR_4_VOX_MAP_HPP__