#include <boost/asio.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>
#include <boost/asio/experimental/channel.hpp>


#include "Utils.h"

namespace PTP
{
    class Client
	{
		struct PtpTimestampSet
		{
			uint16_t sequenceId;
			PtpTimestamp t1; // Master sends Sync (from Follow_Up)
			PtpTimestamp t2; // Slave receives Sync
			PtpTimestamp t3; // Slave sends Delay_Req
			PtpTimestamp t4; // Master receives Delay_Req (from Delay_Resp)
			bool t1Received{ false };
			bool t2Received{ false };
			bool t3Sent{ false };
			bool t4Received{ false };
			std::chrono::steady_clock::time_point creationTime;
		};

	public:

		Client(boost::asio::io_context& ioContext,
			const std::string& serverHost = c_serverIP,
			const std::string& local = c_clientIP);

		Client(const Client&) = delete;
		Client& operator=(const Client&) = delete;
		Client(Client&&) = delete;
		Client& operator=(Client&&) = delete;


	private:

		boost::asio::awaitable<void> ListenOnEventSocket();
		boost::asio::awaitable<void> ListenOnGeneralSocket();
		boost::asio::awaitable<void> RunDelayRequester();
		boost::asio::awaitable<void> CleanupStaleEntries();

		boost::asio::awaitable<void> DelayRequest();

		SimplifiedPtpHeader GetPtpEventHeader();
		SimplifiedPtpHeader GetPtpGeneralHeader();
		PtpTimestamp GetTimeStampFromGeneralBuffer();
		void OnSyncReceived();
		void OnFollowUpReceived();
		void OnRequestResponseReceived();
		void SetupEventSocket(const std::string& serverHost);
		void SetupGeneralSocket(const std::string& serverHost);
		void UpdateMeanPathDelay();
		std::vector<uint8_t> CreateDelayRequest();

		boost::asio::io_context& m_ioContext;
		boost::asio::ip::address m_localAdapter;
		boost::asio::ip::udp::socket m_eventSocket;
		boost::asio::ip::udp::socket m_generalSocket;
		boost::asio::ip::udp::endpoint m_serverEventEndpoint;
		boost::asio::ip::udp::endpoint m_serverGeneralEndpoint;

		std::array<char, 1024> m_eventRecvBuffer{ {} };
		std::array<char, 1024> m_generalRecvBuffer{ {} };

		std::deque<PtpTimestampSet> m_timestampSets;
		std::optional<double> m_meanPathDelay;
		double m_filteredDelay;
		uint16_t m_sequenceId{ 0 };
		// KalmanFilter1D m_kalmanFilter{ 10.0, 1e4/*1.0, 1e4*//*1e-3, 1e3*/ }; // Example process and measurement noise values 
		//AdaptiveKalmanFilter1D m_kalmanFilter{ 10.0 }; // Example process noise and window size
	};
}
