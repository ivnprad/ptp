#include <deque>
#include <cmath>
#include <optional>

namespace PTP
{
    class KalmanFilter1D 
    {
    public:
        KalmanFilter1D(double initialEstimate = 0.0,
                            size_t windowSize = 20,
                            double qScale = 0.01,
                            double qMin = 1e-6,
                            double qMax = 10.0);

        double Update(double measurement);

        // For Unit-tests
        double GetEstimate() const { return m_currentEstimate; }
        double GetMeasurementNoise() const { return m_measurementNoise; }
        double GetProcessNoise() const { return m_processNoise; }
        double GetKalmanGain() const { return m_kalmanGain; }
        double GetEstimateUncertainty() const {return m_estimateUncertainty;}

    private:

        void UpdateProcessNoise();
        void UpdateMeasurementNoise(double measurement);
        void UpdateCovariance();
        void UpdateState(double measurement);
        void ExtrapolateCovariance();
        void CalculateKalmanGain();

        double m_currentEstimate;           // Microseconds
        double m_estimateUncertainty{1.0};  // P
        double m_measurementNoise{1.0};     // R
        double m_processNoise{1.0};         // Q
        double m_kalmanGain{0.0};           // K
        double m_consecutiveHighNIS{0.0};

        std::optional<double> m_prevEstimate; 
        double m_prevEstimateCovariance{0.0};
        size_t m_windowSize;
        double m_qScale, m_qMin, m_qMax;

        std::deque<double> m_measurements;
        std::deque<double> m_innovationHistory;
        std::deque<double> m_nisHistory;
        std::deque<double> m_innoHistory;
        const size_t maxElements{20};

    };
}
