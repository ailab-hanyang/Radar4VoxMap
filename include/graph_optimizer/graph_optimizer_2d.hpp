/**
 * @file graph_optimizer_2d.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>, Jaeyoung Jo <wodud3743@gmail.com>
 * @brief Graph optimizer interface
 * @version 0.1
 * @date 2025-07-05
 *
 * @note This file contains modifications based on the original code from the GitHub repository
 * https://github.com/PRBonn/kiss-icp
 * The original code was contributed by Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and
 * Stachniss, Cyrill and is licensed under the MIT License
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __GRAPH_OPTIMIZER_2D_HPP__
#define __GRAPH_OPTIMIZER_2D_HPP__
#pragma once

#include "graph_optimizer/edge_PV2_DICP.hpp"
#include "graph_optimizer/edge_PV2_PV2_prediction.hpp"
#include "graph_optimizer/edge_PV2_doppler.hpp"
#include "graph_optimizer/vertex_PV2_radar.hpp"

#include "voxeled_rcs_mapper/voxeled_rcs_mapper.hpp"

typedef enum {
    SolverDense2D,
    SolverEigen2D,
    SolverCholmod2D,
    SolverCSparse2D
} OptimizationSolverType2D;

typedef enum {
    GaussNewton2D,
    Levenberge2D
} OptimizationAlgorithmType2D;

#define USE_PV2_VERTEX
typedef VertexPV2Radar      VertexType2D;
typedef EdgePV2PV2Prediction EdgeBinaryPredictionType2D;
typedef EdgePV2DICP         EdgeUnaryICPType2D;
typedef EdgePV2Doppler      EdgeUnaryDopplerType2D;

class GraphOptimizer2D {
public:
    GraphOptimizer2D();
    ~GraphOptimizer2D();

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
    std::vector<Voxel2DHashMap::VoxelBlock> GetAllVoxelFromMap() const {
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
    Eigen::Matrix3d   GetPredictionModel(const VertexType2D* n_1_vertex, const VertexType2D* n_2_vertex, double cur_timestamp) const;
    Eigen::Isometry2d CalculateEdgeError(VertexType2D* vertex1, VertexType2D* vertex2, EdgeBinaryPredictionType2D* edge);

    double GetTransAdaptiveThreshold();
    double GetVelAdaptiveThreshold();
    bool   HasMoved();

private:
    Voxel2DHashMap      m_voxel_hash_map;
    AdaptiveThreshold m_adaptive_threshold;

    std::vector<double>          m_times;
    std::vector<Eigen::Matrix2d> m_poses;

    int m_i_cur_node_idx = 0;
    int m_i_cur_edge_idx = 0;

private:
    // Optimizer
    std::shared_ptr<g2o::SparseOptimizer> m_p_g2o_optimizer;

    VoxelRcsMapperConfig m_config;
};

#endif // __GRAPH_OPTIMIZER_HPP__
