/**
 * @file edge_PV2_PV2_prediction.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Binary Edge for PV prediction interface
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV2_PV2_PREDICTION_HPP__
#define __EDGE_PV2_PV2_PREDICTION_HPP__

#include <graph_optimizer/vertex_PV2_radar.hpp>

class EdgePV2PV2Prediction: public g2o::BaseBinaryEdge<PV2_STATE_SIZE, g2o::VectorN<PV2_STATE_SIZE>, VertexPV2Radar, VertexPV2Radar> {
public:
    EdgePV2PV2Prediction();
    ~EdgePV2PV2Prediction() {}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void                computeError();
    g2o::VectorN<PV2_STATE_SIZE> Prediction(const g2o::VectorN<PV2_STATE_SIZE>& state, double dt);
    // virtual void                linearizeOplus() override;

    virtual void           setMeasurement(const g2o::VectorN<PV2_STATE_SIZE>& measurement);
    virtual void           setMeasurement(const g2o::Isometry2& measurement);
    virtual g2o::Isometry2 measurement() const;

    virtual void setInformation(const g2o::MatrixN<PV2_STATE_SIZE>& information);
    virtual void setInformation(const std::vector<double>& information);
    virtual void getInformation(std::vector<double>& information) const;

    virtual g2o::MatrixN<PV2_STATE_SIZE> informationM6() const;

    Eigen::Matrix<double, PV2_STATE_SIZE, PV2_STATE_SIZE> information_wo_dt;
};

#endif // __EDGE_PV2_PV2_PREDICTION_HPP__