#include "KalmanFilter1D.h"

#include <numeric>
#include <algorithm>
#include <iostream>

namespace PTP
{
	namespace
	{
		double Mean(const std::vector<double>& v)
		{
			return  std::accumulate(v.begin(), v.end(), 0.0)
                        / v.size();
		}
	}

	KalmanFilter1D::KalmanFilter1D(double initialEstimate)
		: m_currentEstimate(initialEstimate)
	{}

	double KalmanFilter1D::Update(double measurement)
	{
		UpdateMeasurementNoise(measurement);
		UpdateProcessNoise();
		ExtrapolateCovariance();
		CalculateKalmanGain();
		UpdateState(measurement);
		UpdateCovariance();
		UpdateHistory(measurement);

		std::cout << std::format(
			"Raw: {:.3f} us | Estimate: {:.3f} us | Q: {:.6f} | K: {:.7f} | R: {:.7f} | Innovation mean:{:.3f} | NIS mean:{:.3f} \r\n",
			measurement,
			m_currentEstimate,
			m_processNoise,
			GetKalmanGain(),
			GetMeasurementNoise(),
			Mean(m_innoHistory),
			Mean(m_nisHistory)
		);

		return m_currentEstimate;
	}

	// Estimate Q from estimate change (system dynamics)
	void KalmanFilter1D::UpdateProcessNoise()
	{
		const auto updateProcessNoise{ [this](auto prevEstimate)
		{
            constexpr auto c_qScale = 0.01;
            constexpr auto c_qMin = 1e-6;
		    constexpr auto c_qMax = 10.0;
			const double delta{ std::abs(m_currentEstimate - prevEstimate) };
			m_processNoise = std::clamp(c_qScale * delta * delta, c_qMin, c_qMax);
			return m_currentEstimate;
		} };

		m_prevEstimate = m_prevEstimate
			.transform(updateProcessNoise)
			.value_or(m_currentEstimate);
	}

	void KalmanFilter1D::UpdateMeasurementNoise(double measurement)
	{
		m_measurements.push_back(measurement);
		const auto size{ m_measurements.size() };
		constexpr auto c_windowSize{ 20 };
		if (size > c_windowSize)
			m_measurements.pop_front();

		if (size < 2)
			return;

		const double mean = std::accumulate(m_measurements.begin(), m_measurements.end(), 0.0)
			/ size;
		const double newVariance = std::accumulate(
			m_measurements.begin(), m_measurements.end(), 0.0,
			[mean](double acc, double v) { return acc + (v - mean) * (v - mean); }
		) / (size - 1);

		m_measurementNoise = std::max(newVariance, 1e-6/* Ensure it is not zero*/);
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
		// Inflate uncertainty if it is too low
		if (m_estimateUncertainty < m_measurementNoise * 0.1)
			m_estimateUncertainty *= 10.0;

		const double ratio{ m_estimateUncertainty / m_measurementNoise };
		m_kalmanGain = ratio / (1.0 + ratio);
	}

	void KalmanFilter1D::UpdateHistory(double measurement)
	{
		constexpr auto c_historyMax{ 50 };
		const double innovation = measurement - m_currentEstimate;
		m_innoHistory.push_back(innovation);
		if (m_innoHistory.size() > c_historyMax)
			m_innoHistory.erase(m_innoHistory.begin());

		const double S = m_estimateUncertainty + m_measurementNoise;
		const double nis = (innovation * innovation) / S;
		m_nisHistory.push_back(nis);
		if (m_nisHistory.size() > c_historyMax)
			m_nisHistory.erase(m_nisHistory.begin());
	}
}