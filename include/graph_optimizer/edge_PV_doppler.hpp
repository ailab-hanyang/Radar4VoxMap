/**
 * @file edge_PV_doppler.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for doppler measurement
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV_DOPPLER_HPP__
#define __EDGE_PV_DOPPLER_HPP__

#include "utils.hpp"
#include <graph_optimizer/vertex_PV_radar.hpp>

class EdgePVDoppler: public g2o::BaseUnaryEdge<3, g2o::Vector3, VertexPVRadar> {
public:
    EdgePVDoppler();
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void computeError();
    // virtual void linearizeOplus();

    virtual void setMeasurement(const g2o::Vector3& measurement);
    virtual void getMeasurement(g2o::Vector3& measurement) const;

    virtual void setInformation(const g2o::MatrixN<3>& information);
    virtual void setInformationScale(const double information);
    virtual void getInformation(std::vector<double>& information) const;

    virtual bool setPointMeasurementAndInformation(const std::vector<SRadarPoint>& points);

    Eigen::Matrix<double, 3, 3> information_from_points;
    double information_scale;
};

#endif // __EDGE_PV_DOPPLER_HPP__
