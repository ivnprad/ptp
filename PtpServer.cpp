#include "PtpServer.h"
#include <iostream>

namespace PTP
{
	namespace
	{
		constexpr auto c_messageSize{sizeof(SimplifiedPtpHeader) + sizeof(PtpTimestamp) };
	}

	Server::Server(boost::asio::io_context& ioContext,
		const std::string& ipAddress,
		unsigned short eventPort,
		unsigned short generalPort)
		: m_ioContext(ioContext)
		, m_localAdapter(boost::asio::ip::make_address(ipAddress))
		, m_eventSocket(ioContext, boost::asio::ip::udp::endpoint(
			boost::asio::ip::udp::v4(), eventPort))
		, m_generalSocket(ioContext, boost::asio::ip::udp::endpoint(
			boost::asio::ip::udp::v4(), generalPort))
		, m_remoteEventEndpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4{}, 0))
		, m_remoteGeneralEndpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4{}, 0))
	{
		// Set socket options on the server's sending socket for robust multicast.

		// 1. Enable loopback so client/server on the same machine can communicate.
		//    This is essential for local testing.
		//m_eventSocket.set_option(boost::asio::ip::multicast::enable_loopback(true));
		m_eventSocket.set_option(boost::asio::ip::multicast::outbound_interface(m_localAdapter.to_v4()));
		m_generalSocket.set_option(boost::asio::ip::multicast::outbound_interface(m_localAdapter.to_v4()));
	
		std::cout << "PTP Server listening on Event Port: "
			<< eventPort << " and General Port: " << generalPort << std::endl;
	

		boost::asio::co_spawn(m_ioContext, Broadcast(), RethrowException);
		boost::asio::co_spawn(m_ioContext, Receive(), RethrowException);
	}

	boost::asio::awaitable<void> Server::Broadcast()
	{
		while (true)
		{
			co_await WaitForTimeout(c_brodcastTimeout);
			co_await SendSyncMessage();
			co_await SendFollowUpMessage();
			++m_sequenceId; // TODO Iher: assuming all clients synchronize within 4 seconds
		}
	}


	boost::asio::awaitable<void> Server::Receive()
	{
		while (true)
		{
			std::vector<uint8_t> buffer(c_messageSize);
			boost::asio::ip::udp::endpoint remoteEndpoint;

			co_await m_eventSocket.async_receive_from(
				boost::asio::buffer(buffer),
				remoteEndpoint,
				boost::asio::use_awaitable);
			
			const auto requestTimeStamp = GetCurrentPtpTime();
			boost::asio::co_spawn(m_ioContext,
				SendDelayResponse(requestTimeStamp, std::move(buffer), std::move(remoteEndpoint)),
				RethrowException);
		}

	}

	

	boost::asio::awaitable<void> Server::SendSyncMessage()
	{
		try
		{
			const boost::asio::ip::udp::endpoint multicastEndpoint{ m_localAdapter.is_loopback()?
				boost::asio::ip::make_address(c_clientIP):c_multicastEvent, c_ptpEventPort };
			const std::vector<uint8_t> syncBuffer{ CreateSyncMessage() };
			m_syncTimestamp = GetCurrentPtpTime();
			const size_t bytesSent
			{
				co_await m_eventSocket.async_send_to(
					boost::asio::buffer(syncBuffer),
					multicastEndpoint,
					boost::asio::use_awaitable)
			};

			if (bytesSent != syncBuffer.size())
			{
				std::cerr << "Failed to send sync message, sent bytes: "
					<< bytesSent << ", expected: " << syncBuffer.size() << std::endl;
			}
		}
		catch (const std::exception& e)
		{
			std::cerr << "Error in server sync loop: " << e.what() << std::endl;
		}
	}

	boost::asio::awaitable<void> Server::SendFollowUpMessage()
	{
		try
		{
			const boost::asio::ip::udp::endpoint multicastEndpoint{ m_localAdapter.is_loopback()?
				boost::asio::ip::make_address(c_clientIP):c_multicastGeneral, c_ptpGeneralPort };
			const std::vector<uint8_t> buffer{ CreateFollowUpMessage() };
			const size_t bytesSent
			{
				co_await m_generalSocket.async_send_to(
					boost::asio::buffer(buffer),
					multicastEndpoint,
					boost::asio::use_awaitable)
			};

			if (bytesSent != buffer.size())
			{
				std::cerr << "Failed to send followup message, sent bytes: "
					<< bytesSent << ", expected: " << buffer.size() << std::endl;
			}
		}
		catch (const std::exception& e)
		{
			std::cerr << "Error in server followup loop: " << e.what() << std::endl;
		}
	}

	std::vector<uint8_t> Server::CreateSyncMessage()
	{
		// A standard Sync message contains a 10-byte originTimestamp in its body.
		std::vector<uint8_t> buffer(c_messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* timestampPtr = reinterpret_cast<PtpTimestamp*>(buffer.data() + sizeof(SimplifiedPtpHeader));
		*timestampPtr = { 0, 0 };

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Sync);
		headerPtr->sequenceId = SwapEndianness(m_sequenceId);
		return buffer;
	}

	std::vector<uint8_t> Server::CreateFollowUpMessage()
	{
		std::vector<uint8_t> buffer(c_messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* timestampPtr = reinterpret_cast<PtpTimestamp*>(buffer.data() + sizeof(SimplifiedPtpHeader));
		*timestampPtr = m_syncTimestamp;

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Follow_Up);
		headerPtr->sequenceId = SwapEndianness(m_sequenceId);
		return buffer;
	}

	boost::asio::awaitable<void> Server::SendDelayResponse(
		PtpTimestamp requestTimeStamp,
		std::vector<uint8_t> receiveBuffer,
		boost::asio::ip::udp::endpoint endpoint)
	{
		// The handler coroutine (SendDelayResponse) does not have a try/catch block. 
		// If an exception is thrown here, it may terminate the coroutine and potentially the process, depending on the coroutine framework. 
		// Consider adding error handling/logging.
		std::vector<uint8_t> sendBuffer{ CreateDelayResponseMessage(requestTimeStamp, receiveBuffer) };
		const boost::asio::ip::udp::endpoint responseEndpoint(
			endpoint.address(),
			c_ptpGeneralPort
		);

		co_await m_generalSocket.async_send_to(
			boost::asio::buffer(sendBuffer, sendBuffer.size()),
			responseEndpoint,
			boost::asio::use_awaitable);
	}

	std::vector<uint8_t> Server::CreateDelayResponseMessage(PtpTimestamp requestTimeStamp, std::vector<uint8_t> receiveBuffer)
	{
		SimplifiedPtpHeader receiveHeader;
		std::memcpy(&receiveHeader, receiveBuffer.data(), sizeof(SimplifiedPtpHeader));

		std::vector<uint8_t> buffer(c_messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* timestampPtr = reinterpret_cast<PtpTimestamp*>(buffer.data() + sizeof(SimplifiedPtpHeader));
		*timestampPtr = requestTimeStamp;

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Delay_Resp);
		headerPtr->sequenceId = receiveHeader.sequenceId;
		return buffer;
	}
}