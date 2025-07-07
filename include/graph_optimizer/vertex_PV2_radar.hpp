/**
 * @file vertex_PV2_radar.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief VertexPVRadar interface
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __VERTEX_PV2_RADAR_HPP__
#define __VERTEX_PV2_RADAR_HPP__

#include "g2o/core/base_vertex.h"
#include "utils.hpp"
#include "voxeled_rcs_mapper/voxeled_rcs_mapper_2d.hpp"

// x, y, z, vx, vy, vz, qx, qy, qz, wx, wy, wz
#define PV2_STATE_SIZE 6
#define IDX2_X 0
#define IDX2_Y 1
#define IDX2_VX 2
#define IDX2_VY 3
#define IDX2_YAW 4
#define IDX2_VYAW 5

class VertexPV2Radar: public g2o::BaseVertex<PV2_STATE_SIZE, g2o::VectorN<PV2_STATE_SIZE>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPV2Radar();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update);

    g2o::Isometry2              estimateIso2() const;
    g2o::VectorN<PV2_STATE_SIZE> estimateVec() const;

    virtual void setEstimate(const g2o::Isometry2& pose);
    virtual void setEstimate(const g2o::VectorN<PV2_STATE_SIZE>& pose);

public:
    double                   timestamp;
    std::vector<SRadarPoint> points; // local frame
};

#endif                               // __VERTEX_PV_RADAR_HPP__
