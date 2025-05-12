/**
 * @file edge_PV_DICP.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for DICP
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV_DICP_HPP__
#define __EDGE_PV_DICP_HPP__

#include <graph_optimizer/vertex_PV_radar.hpp>
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/robust_kernel_impl.h"

class EdgePVDICP: public g2o::BaseUnaryEdge<6, g2o::Isometry3, VertexPVRadar> {
public:
    EdgePVDICP();
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void computeError();
    // virtual void linearizeOplus();

    virtual void setMeasurement(const g2o::Isometry3& m);
    virtual void getMeasurement(g2o::Isometry3& measurement) const;

    virtual void setInformation(const g2o::MatrixN<6>& information);
    virtual void getInformation(std::vector<double>& information) const;

    // DICP
    Eigen::Matrix4d AlignCloudsLocalDoppler6DoF(std::vector<SRadarPoint>& source_global, const std::vector<SRadarPoint>& target,
                                                Eigen::Matrix4d& last_icp_pose, const Velocity& sensor_velocity, double trans_th,
                                                double vel_th, VoxelRcsMapperConfig m_config);

    // Local 좌표계의 frame과, voxel mmap, 초기위치 initial_guess
    Eigen::Matrix4d RunRegister(const std::vector<SRadarPoint>& source_local, std::vector<SRadarPoint>& o_frame,
                                const VoxelHashMap& voxel_map, const Eigen::Matrix4d& initial_guess, const Eigen::Matrix4d& last_pose,
                                const double dt, double trans_sigma, double vel_sigma, VoxelRcsMapperConfig m_config);

    g2o::Isometry3 _inverseMeasurement;
    Eigen::Matrix6d _information_local;
    Eigen::Matrix6d _information_scaled_local;
};

#endif // __EDGE_PV_DICP_HPP__
