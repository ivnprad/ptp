#include "KalmanFilter1D.h"

#include <numeric>
#include <algorithm>
#include <iostream>

namespace PTP
{
    KalmanFilter1D::KalmanFilter1D(double initialEstimate ,
        size_t windowSize ,
        double qScale ,
        double qMin ,
        double qMax )
        : m_currentEstimate(initialEstimate)
        , m_windowSize(windowSize)
        , m_qScale(qScale)
        , m_qMin(qMin)
        , m_qMax(qMax) 
        {}

    double KalmanFilter1D::Update(double measurement)
    {
        UpdateMeasurementNoise(measurement);
        UpdateProcessNoise();
        ExtrapolateCovariance();
        CalculateKalmanGain();
        UpdateState(measurement);
        UpdateCovariance();



        const double innovation { measurement - m_currentEstimate};
        const double nis {(innovation * innovation) 
            / (GetEstimateUncertainty() + GetMeasurementNoise())};

        std::cout << std::format(
                "Raw: {:.3f} us | Estimate: {:.3f} us | K: {:.7f} | R: {:.7f} | Q: {:.7f} | P: {:.7f} | NIS:{:.7f} \r\n ",
                measurement, 
                m_currentEstimate, 
                GetKalmanGain(), 
                GetMeasurementNoise(), 
                GetProcessNoise(), 
                GetEstimateUncertainty(),
                nis
            );

        if (nis > 5.0) 
        {
            m_consecutiveHighNIS++;
            if (m_consecutiveHighNIS > 5) 
            {
                std::cerr<<"Kalman filter may be stuck or overly conservative."<<std::endl;
            }
        }
        else 
        {
            m_consecutiveHighNIS = 0;
        }

        return m_currentEstimate;
    }

    // Estimate Q from estimate change (system dynamics)
    void KalmanFilter1D::UpdateProcessNoise()
    {
        const auto updateProcessNoise{[this](auto prevEstimate)
            {
                const double delta {std::abs(m_currentEstimate - prevEstimate)};
                m_processNoise = std::clamp(m_qScale * delta * delta, m_qMin, m_qMax);
                return m_currentEstimate;
            }};

        m_prevEstimate = m_prevEstimate
                        .transform(updateProcessNoise)
                        .value_or(m_currentEstimate);
    }

    void KalmanFilter1D::UpdateMeasurementNoise(double measurement)
    {
        m_measurements.push_back(measurement);
        const auto size {m_measurements.size()};
        if (size > m_windowSize)
            m_measurements.pop_front();
        
        if (size < 2)
            return;

        const double mean = std::accumulate(m_measurements.begin(), m_measurements.end(), 0.0) 
                            / size;
        const double newVariance = std::accumulate(
                m_measurements.begin(), m_measurements.end(), 0.0,
                [mean](double acc, double v) { return acc + (v - mean) * (v - mean); }
            ) / (size - 1);

        // Heuristic values
        constexpr auto minR = 1.0;
        constexpr auto maxR = 5.0; // Lower max-> K higher -> more responsive. Higher max->K lower-> less responsive.
        m_measurementNoise = std::clamp(newVariance, minR, maxR);

    }

    void KalmanFilter1D::UpdateCovariance()
    {
       m_estimateUncertainty *= (1 - m_kalmanGain); // pn,n= (1−Kn)*pn,n−1
    }

    void KalmanFilter1D::UpdateState(double measurement)
    {
        const auto innovation = measurement - m_currentEstimate;
        m_currentEstimate += m_kalmanGain * innovation; 
    }

    // Covariance Extrapolation for constant dynamics,  x=x State Extrapolation constant dynamics
    void KalmanFilter1D::ExtrapolateCovariance()
    {
        m_estimateUncertainty += m_processNoise; // pn+1,n = pn,n+qn
    }

    void KalmanFilter1D::CalculateKalmanGain()
    {
        constexpr auto minRatio{0.1};
        constexpr auto maxRatio{10.0};
        const double ratio {std::clamp(m_estimateUncertainty / m_measurementNoise,
                                    minRatio, 
                                    maxRatio) };
        m_kalmanGain=ratio / (1.0 + ratio);
    }
}