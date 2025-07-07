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

#include "types/radar_point_cloud.hpp"

#include "graph_optimizer/graph_optimizer.hpp"
#include "graph_optimizer/graph_optimizer_2d.hpp"
#include "voxeled_rcs_mapper/voxeled_rcs_mapper.hpp"
#include "voxeled_rcs_mapper/voxeled_rcs_mapper_2d.hpp"
#include "utils.hpp"

class Radar4VoxMapBase {
public:
    virtual void Init(VoxelRcsMapperConfig config) = 0;
    virtual std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> RunCore(std::vector<RadarPoint> points, const double i_radar_timestamp_sec) = 0;
    virtual std::pair<std::vector<double>, std::vector<double>> GetMotion() = 0;
    virtual std::vector<SRadarPoint> LocalMap() const = 0;
    virtual std::vector<SRadarPoint> StaticLocalMap() const = 0;
    virtual std::vector<VoxelHashMap::VoxelBlock> GetAllVoxelFromMap() const {return std::vector<VoxelHashMap::VoxelBlock>();}
    virtual std::vector<Voxel2DHashMap::VoxelBlock> GetAllVoxelFromMap2D() const {return std::vector<Voxel2DHashMap::VoxelBlock>();}
    virtual std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>,std::vector<SVisEdgeUnary>> GetAllGraphElements() const = 0;
    virtual std::vector<SVisEdgeUnaryDoppler> GetVertexDopplerVel() const = 0;
};

class Radar4VoxMap : public Radar4VoxMapBase {
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

class Radar4VoxMap2D : public Radar4VoxMapBase {
    public:
        using RadarPointVector = std::vector<SRadarPoint>;
        using RadarPointVectorTuple = std::tuple<std::vector<SRadarPoint>, std::vector<SRadarPoint>>;
    
    // Birth
    public:
        Radar4VoxMap2D() : m_tbb_arena(tbb::task_arena::automatic), m_tbb_group(), m_motion(std::make_pair(std::vector<double>(6, 0.0), std::vector<double>(6, 0.0))){};
        ~Radar4VoxMap2D() {m_tbb_arena.execute([&] { m_tbb_group.wait(); });};
    
        void Init(VoxelRcsMapperConfig config);
    
    // Core
    public:
        std::pair<GraphOptimizer2D::AlgoResultTuple, GraphOptimizer2D::AlgoResultTuple> RunCore(std::vector<RadarPoint> points, const double i_radar_timestamp_sec);
        std::pair<std::vector<double>, std::vector<double>> GetMotion(){return m_motion;};
        std::vector<Voxel2DHashMap::VoxelBlock> GetAllVoxelFromMap2D() const {return m_graph_optimizer.GetAllVoxelFromMap();};
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
        GraphOptimizer2D m_graph_optimizer;
        // VoxelHashMap m_voxel_hash_map;
    
};

#endif // __RADAR_4_VOX_MAP_HPP__