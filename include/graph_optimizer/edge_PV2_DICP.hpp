/**
 * @file edge_PV2_DICP.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for DICP
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV2_DICP_HPP__
#define __EDGE_PV2_DICP_HPP__

#include <graph_optimizer/vertex_PV2_radar.hpp>
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/robust_kernel_impl.h"

class EdgePV2DICP: public g2o::BaseUnaryEdge<3, g2o::Isometry2, VertexPV2Radar> {
public:
    EdgePV2DICP();
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void computeError();
    // virtual void linearizeOplus();

    virtual void setMeasurement(const g2o::Isometry2& m);
    virtual void getMeasurement(g2o::Isometry2& measurement) const;

    virtual void setInformation(const g2o::MatrixN<3>& information);
    virtual void getInformation(std::vector<double>& information) const;

    // DICP
    Eigen::Isometry2d AlignCloudsLocalDoppler3DoF(std::vector<SRadarPoint>& source_global, const std::vector<SRadarPoint>& target,
                                                 Eigen::Isometry2d& last_icp_pose, const Velocity2D& sensor_velocity, double trans_th,
                                                 double vel_th, VoxelRcsMapperConfig m_config);

    // Local 좌표계의 frame과, voxel mmap, 초기위치 initial_guess
    Eigen::Isometry2d RunRegister(const std::vector<SRadarPoint>& source_local, std::vector<SRadarPoint>& o_frame,
                                  const Voxel2DHashMap& voxel_map, const Eigen::Isometry2d& initial_guess, const Eigen::Isometry2d& last_pose,
                                  const double dt, double trans_sigma, double vel_sigma, VoxelRcsMapperConfig m_config);

    g2o::Isometry2 _inverseMeasurement;
    Eigen::Matrix3d _information_local;
    Eigen::Matrix3d _information_scaled_local;
};

#endif // __EDGE_PV2_DICP_HPP__
