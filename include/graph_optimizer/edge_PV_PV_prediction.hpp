/**
 * @file edge_PV_PV_prediction.hpp
 * @author Jiwon Seok <pauljiwon96@gmail.com>
 * @brief Binary Edge for PV prediction interface
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __EDGE_PV_PV_PREDICTION_HPP__
#define __EDGE_PV_PV_PREDICTION_HPP__

#include <graph_optimizer/vertex_PV_radar.hpp>

class EdgePVPVPrediction: public g2o::BaseBinaryEdge<PV_STATE_SIZE, g2o::VectorN<PV_STATE_SIZE>, VertexPVRadar, VertexPVRadar> {
public:
    EdgePVPVPrediction();
    ~EdgePVPVPrediction() {}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void                computeError();
    g2o::VectorN<PV_STATE_SIZE> Prediction(const g2o::VectorN<PV_STATE_SIZE>& state, double dt);
    // virtual void                linearizeOplus() override;

    virtual void           setMeasurement(const g2o::VectorN<PV_STATE_SIZE>& measurement);
    virtual void           setMeasurement(const g2o::Isometry3& measurement);
    virtual g2o::Isometry3 measurement() const;

    virtual void setInformation(const g2o::MatrixN<PV_STATE_SIZE>& information);
    virtual void setInformation(const std::vector<double>& information);
    virtual void getInformation(std::vector<double>& information) const;

    virtual g2o::MatrixN<6> informationM6() const;

    Eigen::Matrix<double, PV_STATE_SIZE, PV_STATE_SIZE> information_wo_dt;
};

#endif // __EDGE_PV_PV_PREDICTION_HPP__