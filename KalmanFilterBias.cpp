#include "KalmanFilterBias.h"
#include <format>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <cmath>

KalmanFilterBias::KalmanFilterBias(double initialEstimate, double initialUncertainty, double processNoise, double measurementNoise)
    : m_currentEstimate(initialEstimate),
      m_estimateUncertainty(initialUncertainty),
      m_processNoise(processNoise),
      m_measurementNoise(measurementNoise),
      m_kalmanGain(0.0),
      m_prevEstimate(initialEstimate)
{
}

double KalmanFilterBias::GetEstimate() const
{
    return m_currentEstimate;
}

double KalmanFilterBias::GetKalmanGain() const
{
    return m_kalmanGain;
}

double KalmanFilterBias::GetMeasurementNoise() const
{
    return m_measurementNoise;
}

double KalmanFilterBias::Update(double measurement)
{
    ExtrapolateCovariance();
    CalculateKalmanGain();

    const double innovation = measurement - (m_currentEstimate + m_bias);
    m_innoHistory.push_back(innovation);
    if (m_innoHistory.size() > 50) m_innoHistory.erase(m_innoHistory.begin());

    const double S = m_estimateUncertainty + m_biasUncertainty + m_measurementNoise;
    const double nis = (innovation * innovation) / S;
    m_nisHistory.push_back(nis);
    if (m_nisHistory.size() > 50) m_nisHistory.erase(m_nisHistory.begin());

    UpdateState(measurement);
    UpdateCovariance();
    UpdateProcessNoise();

    //UpdateMeasurementNoise();
    auto mean = [](const std::vector<double>& v) {
        return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    double meanNIS = mean(m_nisHistory);
    if (meanNIS > 0.01 && meanNIS < 100) {
        m_measurementNoise *= meanNIS;
        m_measurementNoise = std::clamp(m_measurementNoise, 1.0, 100.0);
        //m_measurementNoise = std::clamp(m_measurementNoise, 1e-2, 1e3);
    }

    //Log(measurement);

    std::cout << std::format(
        "Raw: {:.3f} us | Estimate: {:.3f} us | Bias: {:.3f} | Q: {:.6f} | K: {:.7f} | R: {:.7f} | Innovation mean:{:.3f} | NIS mean:{:.3f} \r\n",
        measurement,
        m_currentEstimate,
        m_bias,
        m_processNoise,
        GetKalmanGain(),
        GetMeasurementNoise(),
        mean(m_innoHistory),
        meanNIS 
    );

    return m_currentEstimate;
}

void KalmanFilterBias::ExtrapolateCovariance()
{
    m_estimateUncertainty += m_processNoise;
    m_biasUncertainty += m_biasProcessNoise;
}

void KalmanFilterBias::CalculateKalmanGain()
{
    const double S = m_estimateUncertainty + m_biasUncertainty + m_measurementNoise;
    m_kalmanGain = m_estimateUncertainty / S;
    m_biasKalmanGain = m_biasUncertainty / S;
}

void KalmanFilterBias::UpdateState(double measurement)
{
    const double innovation = measurement - (m_currentEstimate + m_bias);
    m_currentEstimate += m_kalmanGain * innovation;
    m_bias += m_biasKalmanGain * innovation;
}

void KalmanFilterBias::UpdateCovariance()
{
    m_estimateUncertainty *= (1.0 - m_kalmanGain);
    m_biasUncertainty *= (1.0 - m_biasKalmanGain);
}

void KalmanFilterBias::UpdateProcessNoise()
{
    const auto updateProcessNoise = [this](double prevEstimate) {
        const double delta = std::abs(m_currentEstimate - prevEstimate);
        m_processNoise = std::clamp(m_qScale * delta * delta, m_qMin, m_qMax);
        return m_currentEstimate;
    };

    m_prevEstimate = m_prevEstimate
        .transform(updateProcessNoise)
        .value_or(m_currentEstimate);
}

void KalmanFilterBias::UpdateMeasurementNoise()
{
    // Estimate R after P and Pb have been updated
    if (m_innoHistory.size() >= 10) {
        double mean_y2 = std::accumulate(m_innoHistory.begin(), m_innoHistory.end(), 0.0,
                                        [](double acc, double y) { return acc + y * y; }) / m_innoHistory.size();

        const double inferredR = mean_y2 - m_estimateUncertainty - m_biasUncertainty;
        m_measurementNoise = std::clamp(inferredR, 1e-3, 1e3);  // Use a tighter clamp to prevent runaway growth
    }
}

void KalmanFilterBias::Log(double measurement)
{
    auto mean = [](const std::vector<double>& v) {
        return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };
    std::cout << std::format(
        "Raw: {:.3f} us | Estimate: {:.3f} us | Bias: {:.3f} | Q: {:.6f} | K: {:.7f} | R: {:.7f} | Innovation mean:{:.3f} | NIS mean:{:.3f} \r\n",
        measurement,
        m_currentEstimate,
        m_bias,
        m_processNoise,
        GetKalmanGain(),
        GetMeasurementNoise(),
        mean(m_innoHistory),
        mean(m_nisHistory)  
    );
}
