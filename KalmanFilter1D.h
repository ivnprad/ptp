#include <deque>
#include <cmath>
#include <optional>
#include <vector>

namespace PTP
{
    class KalmanFilter1D
	{
	public:
		KalmanFilter1D(double initialEstimate = 300.0/*heuristic Value us*/);

		double Update(double measurement);

		// For Unit-tests
		double GetEstimate() const { return m_currentEstimate; }
		double GetMeasurementNoise() const { return m_measurementNoise; }
		double GetProcessNoise() const { return m_processNoise; }
		double GetKalmanGain() const { return m_kalmanGain; }
		double GetEstimateUncertainty() const { return m_estimateUncertainty; }

	private:

		void UpdateProcessNoise();
		void UpdateMeasurementNoise(double measurement);
		void UpdateCovariance();
		void UpdateState(double measurement);
		void ExtrapolateCovariance();
		void CalculateKalmanGain();
		void UpdateHistory(double measurement);

		double m_currentEstimate;           // Microseconds
		double m_estimateUncertainty{ 1.0 };  // P
		double m_measurementNoise{ 1.0 };     // R
		double m_processNoise{ 1.0 };         // Q
		double m_kalmanGain{ 0.0 };           // K

		std::optional<double> m_prevEstimate;

		std::deque<double> m_measurements;
		std::vector<double> m_innoHistory;
		std::vector<double> m_nisHistory;

	};
}
