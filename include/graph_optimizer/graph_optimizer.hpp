/**
 * @file graph_optimizer.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>, Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Graph optimizer interface
 * @version 0.1
 * @date 2024-08-23
 *
 * @note This file contains modifications based on the original code from the GitHub repository
 * https://github.com/PRBonn/kiss-icp
 * The original code was contributed by Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and
 * Stachniss, Cyrill and is licensed under the MIT License
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __GRAPH_OPTIMIZER_HPP__
#define __GRAPH_OPTIMIZER_HPP__
#pragma once

#include "graph_optimizer/edge_PV_DICP.hpp"
#include "graph_optimizer/edge_PV_PV_prediction.hpp"
#include "graph_optimizer/edge_PV_doppler.hpp"
#include "graph_optimizer/vertex_PV_radar.hpp"
#include "graph_optimizer/edge_PV_GravityAlign.hpp"

#include "voxeled_rcs_mapper/voxeled_rcs_mapper.hpp"

typedef enum {
    SolverDense,
    SolverEigen,
    SolverCholmod,
    SolverCSparse
} OptimizationSolverType;

typedef enum {
    GaussNewton,
    Levenberge
} OptimizationAlgorithmType;

#define USE_PV_VERTEX
typedef VertexPVRadar      VertexType;
typedef EdgePVPVPrediction EdgeBinaryPredictionType;
typedef EdgePVDICP         EdgeUnaryICPType;
typedef EdgePVDoppler      EdgeUnaryDopplerType;
typedef EdgePVGravityAlign EdgeUnaryGravityAlignType;

class GraphOptimizer {
public:
    GraphOptimizer();
    ~GraphOptimizer();

    using AlgoResultTuple = std::tuple<std::vector<SRadarPoint>, Eigen::Matrix4d, double>;
    using PairAlgoResultTuple = std::pair<AlgoResultTuple, AlgoResultTuple>;

    void Init(VoxelRcsMapperConfig config);

    void                AddNewFrame(const std::vector<RadarPoint>& frame, const double i_radar_timestamp_sec);
    void                RunIcpAll();
    void                RunOptimization();
    void                UpdateGraph();
    void                UpdateVoxelMap();
    PairAlgoResultTuple GetLastVertexInfo();
    std::pair<std::vector<double>, std::vector<double>> GetMotion();

public:
    std::vector<VoxelHashMap::VoxelBlock> GetAllVoxelFromMap() const {
        return m_voxel_hash_map.GetAllVoxel();
    }; // Return all voxel blocks in the voxel map
    std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>, std::vector<SVisEdgeUnary>> GetAllGraphElements() const;
    std::vector<SVisEdgeUnaryDoppler> GetVertexDopplerVel() const;

    std::vector<SRadarPoint> LocalMap() const { return m_voxel_hash_map.Pointcloud(); };
    std::vector<SRadarPoint> StaticLocalMap() const { return m_voxel_hash_map.StaticPointcloud(); };

private:
    int GetNewNodeIndex() { return ++m_i_cur_node_idx; };
    int GetNewEdgeIndex() { return ++m_i_cur_edge_idx; };

    void UpdateNodeIndex() { m_i_cur_node_idx++; };
    void UpdateEdgeIndex() { m_i_cur_edge_idx++; };

    int GetCurNodeIndex() const { return m_i_cur_node_idx; };
    int GetCurEdgeIndex() const { return m_i_cur_edge_idx; };

    // Functions
private:
    Eigen::Matrix4d   GetPredictionModel(const VertexType* n_1_vertex, const VertexType* n_2_vertex, double cur_timestamp) const;
    Eigen::Isometry3d CalculateEdgeError(VertexType* vertex1, VertexType* vertex2, EdgeBinaryPredictionType* edge);

    double GetTransAdaptiveThreshold();
    double GetVelAdaptiveThreshold();
    bool   HasMoved();

private:
    VoxelHashMap      m_voxel_hash_map;
    AdaptiveThreshold m_adaptive_threshold;

    std::vector<double>          m_times;
    std::vector<Eigen::Matrix4d> m_poses;

    int m_i_cur_node_idx = 0;
    int m_i_cur_edge_idx = 0;

private:
    // Optimizer
    std::shared_ptr<g2o::SparseOptimizer> m_p_g2o_optimizer;

    VoxelRcsMapperConfig m_config;
};

#endif // __GRAPH_OPTIMIZER_HPP__
