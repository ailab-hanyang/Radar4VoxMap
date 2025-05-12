/**
 * @file vertex_PV_radar.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief VertexPVRadar interface
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __VERTEX_PV_RADAR_HPP__
#define __VERTEX_PV_RADAR_HPP__

#include "g2o/core/base_vertex.h"
#include "utils.hpp"
#include "voxeled_rcs_mapper/voxeled_rcs_mapper.hpp"

// x, y, z, vx, vy, vz, qx, qy, qz, wx, wy, wz
#define PV_STATE_SIZE 12
#define IDX_X 0
#define IDX_Y 1
#define IDX_Z 2
#define IDX_VX 3
#define IDX_VY 4
#define IDX_VZ 5
#define IDX_QX 6
#define IDX_QY 7
#define IDX_QZ 8
#define IDX_WX 9
#define IDX_WY 10
#define IDX_WZ 11

class VertexPVRadar: public g2o::BaseVertex<PV_STATE_SIZE, g2o::VectorN<PV_STATE_SIZE>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPVRadar();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update);

    g2o::Isometry3              estimateIso3() const;
    g2o::VectorN<PV_STATE_SIZE> estimateVec() const;

    virtual void setEstimate(const g2o::Isometry3& pose);
    virtual void setEstimate(const g2o::VectorN<PV_STATE_SIZE>& pose);

public:
    double                   timestamp;
    std::vector<SRadarPoint> points; // local frame
};

#endif                               // __VERTEX_PV_RADAR_HPP__
