/**
 * @file edge_PV2_doppler.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Edge for doppler measurement
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV2_DOPPLER_HPP__
#define __EDGE_PV2_DOPPLER_HPP__

#include "utils.hpp"
#include <graph_optimizer/vertex_PV2_radar.hpp>

class EdgePV2Doppler: public g2o::BaseUnaryEdge<2, g2o::Vector2, VertexPV2Radar> {
public:
    EdgePV2Doppler();
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void computeError();
    // virtual void linearizeOplus();

    virtual void setMeasurement(const g2o::Vector2& measurement);
    virtual void getMeasurement(g2o::Vector2& measurement) const;

    virtual void setInformation(const g2o::MatrixN<2>& information);
    virtual void setInformationScale(const double information);
    virtual void getInformation(std::vector<double>& information) const;

    virtual bool setPointMeasurementAndInformation(const std::vector<SRadarPoint>& points);

    static std::vector<SRadarPoint> ExtractStaticPoints(const std::vector<SRadarPoint>& points, const Eigen::Vector2d& velocity);

    Eigen::Matrix<double, 2, 2> information_from_points;
    double information_scale;
};

#endif // __EDGE_PV2_DOPPLER_HPP__
