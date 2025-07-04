/**
 * @file edge_PV_GravityAlign.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for Gravity Align
 * @version 0.1
 * @date 2025-05-15
 *
 * @copyright Copyright (c) 2025
 */

#ifndef __EDGE_PV_GRAVITYALIGN_HPP__
#define __EDGE_PV_GRAVITYALIGN_HPP__

#include <graph_optimizer/vertex_PV_radar.hpp>
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/robust_kernel_impl.h"
#include "utils.hpp"

class EdgePVGravityAlign: public g2o::BaseUnaryEdge<3, g2o::Vector3, VertexPVRadar> {
public:
    EdgePVGravityAlign(){}
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void computeError(){
        // 오차 계산
        Eigen::Vector3d gravity_vector_normalized = Eigen::Vector3d(0, 0, -1);

        // Get measurement from state roll, pitch
        VertexPVRadar* vertex = static_cast<VertexPVRadar*>(_vertices[0]);
        auto iso3 = vertex->estimateIso3();
        Eigen::Matrix3d rotation_matrix = iso3.rotation();
        Eigen::Vector3d local_down_vector(0, 0, -1); // 로컬 좌표계에서 아래 방향
        Eigen::Vector3d world_down_vector = rotation_matrix * local_down_vector;

        _error = gravity_vector_normalized - world_down_vector;
    }
    // virtual void linearizeOplus();

    // virtual void setMeasurement(const g2o::Isometry3& m);
    // virtual void getMeasurement(g2o::Isometry3& measurement) const;

    // virtual void setInformation(const g2o::MatrixN<6>& information);
    // virtual void getInformation(std::vector<double>& information) const;
    virtual void setInformation(const double information) {
        _information = information*Eigen::Matrix3d::Identity();
    }
};

#endif // __EDGE_PV_GRAVITYALIGN_HPP__
