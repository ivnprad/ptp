#pragma once

#include <vector>
#include <optional>

class KalmanFilterBias
{
public:
KalmanFilterBias(double initialEstimate=0.0,
     double initialUncertainty=1000.0/*1.0*/,
      double processNoise=0.1/*1.0*/, 
      double measurementNoise=1.0);

    double Update(double measurement);
    double GetEstimate() const;
    double GetKalmanGain() const;
    double GetMeasurementNoise() const;

private:
    void ExtrapolateCovariance();
    void CalculateKalmanGain();
    void UpdateState(double measurement);
    void UpdateCovariance();
    void UpdateProcessNoise();
    void UpdateMeasurementNoise();
    void Log(double measurement);

    double m_currentEstimate;
    std::optional<double> m_prevEstimate;
    double m_estimateUncertainty;
    double m_processNoise;
    double m_measurementNoise;
    double m_kalmanGain;

    // ➕ Bias tracking
    double m_bias = 0.0;
    double m_biasUncertainty = 1.0;
    double m_biasProcessNoise = 1e-6;
    double m_biasKalmanGain = 0.0;

    // ⚙️ Adaptive Q tuning
    double m_qScale = 0.1/*1.0*/;
    double m_qMin = 1e-6;
    double m_qMax = 1.0/*10.0*/;

    std::vector<double> m_innoHistory;
    std::vector<double> m_nisHistory;
};
