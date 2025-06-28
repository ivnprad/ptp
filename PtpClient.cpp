#include "PtpClient.h"
#include <range/v3/all.hpp> 

#include <iostream>

namespace PTP
{
	Client::Client(boost::asio::io_context& ioContext,
		const std::string& serverHost,
		const std::string& local)
		: m_ioContext(ioContext)
		, m_localAdapter(boost::asio::ip::make_address(local))
		, m_eventSocket(m_ioContext)
		, m_generalSocket(m_ioContext)
	{
		try
		{
			SetupEventSocket(serverHost);
			SetupGeneralSocket(serverHost);
			boost::asio::co_spawn(m_ioContext, ListenOnEventSocket(), RethrowException);
			boost::asio::co_spawn(m_ioContext, ListenOnGeneralSocket(), RethrowException);
			boost::asio::co_spawn(m_ioContext, RunDelayRequester(), RethrowException);
			boost::asio::co_spawn(m_ioContext, CleanupStaleEntries(), RethrowException);
		}
		catch (const std::exception& e)
		{
			std::cerr << "Error resolving server endpoint: " << e.what() << std::endl;
			throw;
		}
	}

    boost::asio::awaitable<void> Client::ListenOnEventSocket()
	{
		while (true)
		{
			boost::asio::ip::udp::endpoint senderEndpoint;
			co_await m_eventSocket.async_receive_from(
				boost::asio::buffer(m_eventRecvBuffer),
				senderEndpoint,
				boost::asio::use_awaitable);
			OnSyncReceived();
		}
	}

    boost::asio::awaitable<void> Client::ListenOnGeneralSocket()
	{
		while (true)
		{
			boost::asio::ip::udp::endpoint senderEndpoint;
			co_await m_generalSocket.async_receive_from(
				boost::asio::buffer(m_generalRecvBuffer),
				senderEndpoint,
				boost::asio::use_awaitable);

			OnFollowUpReceived();
			OnRequestResponseReceived();
		}
	}

    boost::asio::awaitable<void> Client::RunDelayRequester()
	{
		while (true)
		{
			co_await WaitForTimeout(c_delayRequestTimeout);
			co_await DelayRequest();
			// Note: We do not wait for the response here, as the ListenOnGeneralSocket will handle it.
		}
	}

    boost::asio::awaitable<void> Client::CleanupStaleEntries()
	{
		while (true)
		{
			co_await WaitForTimeout(c_cleanupInterval);
			const auto now = std::chrono::steady_clock::now();

			// Remove any entries that are older than the timeout AND are not yet complete.
			// This handles cases where a Follow_Up or Delay_Resp was lost.
			const auto entriesBeforeCleanup = m_timestampSets.size();
			const auto numStale = std::erase_if(m_timestampSets, [&](const PtpTimestampSet& entry)
			{
				const bool isComplete = entry.t1Received && entry.t2Received && entry.t3Sent && entry.t4Received;
				if (isComplete)
				{
					return false; // Don't remove completed entries based on time.
				}
				return (now - entry.creationTime) > c_entryStaleTimeout;
			});

			if (numStale > 0)
			{
				std::cout << std::format("entries before {}. Cleanup task removed {} stale PTP entries. Entries left: {}",
					entriesBeforeCleanup,
					numStale,
					m_timestampSets.size()) << std::endl;
			}

			while (m_timestampSets.size() > c_maxTimestampSets)
			{
				m_timestampSets.pop_front();// Keep only the last 10 timestamp sets
			}
		}
	}

    boost::asio::awaitable<void> Client::DelayRequest()
	{
		const auto buffer = CreateDelayRequest();
		co_await m_eventSocket.async_send_to(
			boost::asio::buffer(buffer, buffer.size()),
			m_serverEventEndpoint,
			boost::asio::use_awaitable);
	}


	SimplifiedPtpHeader Client::GetPtpEventHeader()
	{
		SimplifiedPtpHeader header;
		std::memcpy(&header, m_eventRecvBuffer.data(), sizeof(SimplifiedPtpHeader));
		return header;
	}

	SimplifiedPtpHeader Client::GetPtpGeneralHeader()
	{
		SimplifiedPtpHeader header;
		std::memcpy(&header, m_generalRecvBuffer.data(), sizeof(SimplifiedPtpHeader));
		return header;
	}

	PtpTimestamp Client::GetTimeStampFromGeneralBuffer()
	{
		PtpTimestamp timestampPayload;
		std::memcpy(&timestampPayload, m_generalRecvBuffer.data() + sizeof(SimplifiedPtpHeader), sizeof(PtpTimestamp));
		return timestampPayload;
	}

	void Client::OnSyncReceived()
	{
		const auto t2{ GetCurrentPtpTime() };
		const auto ptpHeader{ GetPtpEventHeader() };
		if (ptpHeader.GetMessageType() != PtpMessageType::Sync)
			return;

		PtpTimestampSet newSet;
		m_sequenceId = SwapEndianness(ptpHeader.sequenceId);
		newSet.sequenceId = m_sequenceId;
		newSet.t2 = t2;
		newSet.t2Received = true;
		newSet.creationTime = std::chrono::steady_clock::now();

		m_timestampSets.push_back(newSet);
	}

	void Client::OnFollowUpReceived()
	{
		const auto ptpHeader{ GetPtpGeneralHeader() };
		if (ptpHeader.GetMessageType() != PtpMessageType::Follow_Up)
			return;

		const auto OnSequenceId = [this](PtpTimestampSet ptpTimestampSet)
		{
			return ptpTimestampSet.sequenceId == m_sequenceId;
		};

		for (auto& ptpTimestampSet : m_timestampSets | ranges::views::filter(OnSequenceId))
		{
			ptpTimestampSet.t1 = GetTimeStampFromGeneralBuffer();
			ptpTimestampSet.t1Received = true;
		}
	}

	void Client::OnRequestResponseReceived()
	{
		const auto ptpHeader{ GetPtpGeneralHeader() };
		if (ptpHeader.GetMessageType() != PtpMessageType::Delay_Resp)
			return;

		const auto OnSequenceId = [this](PtpTimestampSet ptpTimestampSet)
		{
			return ptpTimestampSet.sequenceId == m_sequenceId;
		};
		for (auto& ptpTimestampSet : m_timestampSets | ranges::views::filter(OnSequenceId))
		{
			ptpTimestampSet.t4 = GetTimeStampFromGeneralBuffer();
			ptpTimestampSet.t4Received = true;
		}

		UpdateMeanPathDelay();

	}
	void Client::SetupEventSocket(const std::string& serverHost)
	{
		boost::asio::ip::udp::resolver resolver(m_ioContext);
		m_serverEventEndpoint = *resolver.resolve(
			boost::asio::ip::udp::v4(),
			serverHost,
			std::to_string(c_ptpEventPort)).begin();
		m_eventSocket.open(boost::asio::ip::udp::v4());
		m_eventSocket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		std::cout << "PTP Client Event will send unicast to: " << m_serverEventEndpoint << std::endl;

		const boost::asio::ip::udp::endpoint localListenEndpoint(
            boost::asio::ip::make_address(c_clientIP), c_ptpEventPort);
		boost::system::error_code ec{};
		m_eventSocket.bind(localListenEndpoint, ec);
        if (ec)
            throw std::runtime_error("Failed to bind event socket");

		if (!m_localAdapter.is_loopback())
		{
		  m_eventSocket.set_option(
		  	boost::asio::ip::multicast::join_group(c_multicastEvent, m_localAdapter.to_v4()));
		 std::cout << "Client Event joined multicast group " << c_multicastEvent.to_string()
		 	<< " on interface " << m_localAdapter.to_string() << std::endl;
		}
	}

	void Client::SetupGeneralSocket(const std::string& serverHost)
	{
		boost::asio::ip::udp::resolver resolver(m_ioContext);
		m_serverGeneralEndpoint = *resolver.resolve(
			boost::asio::ip::udp::v4(),
			serverHost,
			std::to_string(c_ptpGeneralPort)).begin();
		m_generalSocket.open(boost::asio::ip::udp::v4());
		m_generalSocket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		std::cout << "PTP Client General will send unicast to: " << m_serverGeneralEndpoint << std::endl;

		const boost::asio::ip::udp::endpoint localListenEndpoint(
            boost::asio::ip::make_address(c_clientIP), c_ptpGeneralPort);
		boost::system::error_code ec{};
		m_generalSocket.bind(localListenEndpoint, ec);
        if (ec)
            throw std::runtime_error("Failed to bind general socket"); 
		if (!m_localAdapter.is_loopback())
		{
		 m_generalSocket.set_option(
		  	boost::asio::ip::multicast::join_group(c_multicastGeneral, m_localAdapter.to_v4()));
		 std::cout << "Client General joined multicast group " << c_multicastGeneral.to_string()
		 	<< " on interface " << m_localAdapter.to_string() << std::endl;
		}
	}

	void Client::UpdateMeanPathDelay()
	{

		if (m_timestampSets.empty())
			return;

		const auto isComplete = [](PtpTimestampSet entry)
		{
			return entry.t1Received && entry.t2Received && entry.t3Sent && entry.t4Received;
		};

		const auto calculatePathDelay = [](const PtpTimestampSet& entry)
		{
			const auto t1 = entry.t1.to_nanoseconds();
			const auto t2 = entry.t2.to_nanoseconds();
			const auto t3 = entry.t3.to_nanoseconds();
			const auto t4 = entry.t4.to_nanoseconds();
			return ((t4 - t1) - (t3 - t2)) / 2.0;
		};

		const auto aboveZero = [](const auto pathDelay)
		{
			return pathDelay>0;
		};

		const auto toMicroseconds = [](const auto& value)
		{
			return value / 1000.0;
		};

        auto pathDelays = m_timestampSets
                | ranges::views::filter(isComplete)
                | ranges::views::reverse
                | ranges::views::take(c_maxTimestampSets)
                | ranges::views::transform(calculatePathDelay)
				| ranges::views::filter(aboveZero)
                | ranges::views::transform(toMicroseconds)
                | ranges::to_vector;

		if (!pathDelays.empty())
		{
			const double rawMeasurement=pathDelays.back();
			//m_meanPathDelay = m_kalmanFilter.Update(rawMeasurement);
			m_meanPathDelay = m_kalmanFilterBias.Update(rawMeasurement);
		}
	}

	std::vector<uint8_t> Client::CreateDelayRequest()
	{
		const auto OnSequenceId = [this](PtpTimestampSet ptpTimestampSet)
		{
			return ptpTimestampSet.sequenceId == m_sequenceId;
		};

		for (auto& ptpTimestampSet : m_timestampSets | ranges::views::filter(OnSequenceId))
		{
			ptpTimestampSet.t3 = GetCurrentPtpTime();
			ptpTimestampSet.t3Sent = true;
		}

		constexpr auto messageSize{ sizeof(SimplifiedPtpHeader) + sizeof(PtpTimestamp) };
		std::vector<uint8_t> buffer(messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Delay_Req);
		headerPtr->sequenceId = SwapEndianness(m_sequenceId);
		return buffer;
	}
}