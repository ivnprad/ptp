#include "PtpServer.h"
#include <iostream>

namespace PTP
{
	Server::Server(boost::asio::io_context& ioContext,
		const std::string& ipAddress,
		unsigned short eventPort,
		unsigned short generalPort)
		: m_ioContext(ioContext)
		, m_localAdapter(boost::asio::ip::make_address(ipAddress))
		, m_eventSocket(ioContext, boost::asio::ip::udp::endpoint(
		boost::asio::ip::udp::v4()/*m_localAdapter*/, eventPort))
		, m_generalSocket(ioContext, boost::asio::ip::udp::endpoint(
		boost::asio::ip::udp::v4()/*m_localAdapter*/, generalPort))
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


	// One perpetual Listener task: Its only job is to co_await a new request.
	//	Many temporary Handler tasks : When the Listener gets a request, it doesn't process it directly. Instead, it immediately spawns a new, temporary coroutine to handle that one request. It passes all the necessary information (like the client's endpoint) as parameters to this new handler task.
	//	The Listener immediately loops back to co_await the next request, while the handler task for the first client runs concurrently.
	//This way, the server can accept a new request from Client B while it's still processing the request from Client A, making it truly concurrent and scalable.
	boost::asio::awaitable<void> Server::Receive()
	{
		while (true)
		{
			co_await WaitDelayRequest();
			// When WaitDelayRequest receives a message, it must parse the header and extract the client's sequenceId.
			//This extracted sequenceId must then be passed to SendRequestReponse and CreateDelayResponseMessage.
			//The m_sequenceId member variable should only be touched by the Broadcast task, as it belongs exclusively to that conversation.
			co_await SendRequestReponse();
		}
	}

	boost::asio::awaitable<void> Server::SendSyncMessage()
	{
		try
		{
			const boost::asio::ip::udp::endpoint multicastEndpoint{ c_multicastEvent, c_ptpEventPort };

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
			const boost::asio::ip::udp::endpoint multicastEndpoint{ c_multicastGeneral, c_ptpGeneralPort };

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

	boost::asio::awaitable<void> Server::WaitDelayRequest()
	{
		co_await m_eventSocket.async_receive_from(
			boost::asio::buffer(m_eventRecvBuffer),
			m_remoteEventEndpoint,
			boost::asio::use_awaitable);
		m_requestTimeStamp = GetCurrentPtpTime();
	}

	boost::asio::awaitable<void> Server::SendRequestReponse()
	{
		std::vector<uint8_t> buffer{ CreateDelayResponseMessage() };
		const boost::asio::ip::udp::endpoint responseEndpoint(
			m_remoteEventEndpoint.address(),
			c_ptpGeneralPort
		);

		co_await m_generalSocket.async_send_to(
			boost::asio::buffer(buffer, buffer.size()),
			responseEndpoint,
			boost::asio::use_awaitable);
	}

	std::vector<uint8_t> Server::CreateSyncMessage()
	{
		// A standard Sync message contains a 10-byte originTimestamp in its body.
		constexpr auto syncMessageSize{ sizeof(SimplifiedPtpHeader) + sizeof(PtpTimestamp) };
		std::vector<uint8_t> buffer(syncMessageSize);

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
		constexpr auto messageSize{ sizeof(SimplifiedPtpHeader) + sizeof(PtpTimestamp) };
		std::vector<uint8_t> buffer(messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* timestampPtr = reinterpret_cast<PtpTimestamp*>(buffer.data() + sizeof(SimplifiedPtpHeader));
		*timestampPtr = m_syncTimestamp;

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Follow_Up);
		headerPtr->sequenceId = SwapEndianness(m_sequenceId);
		return buffer;
	}

	std::vector<uint8_t> Server::CreateDelayResponseMessage()
	{
		constexpr auto messageSize{ sizeof(SimplifiedPtpHeader) + sizeof(PtpTimestamp) };
		std::vector<uint8_t> buffer(messageSize);

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* timestampPtr = reinterpret_cast<PtpTimestamp*>(buffer.data() + sizeof(SimplifiedPtpHeader));
		*timestampPtr = m_requestTimeStamp;

		#pragma warning(suppress: 26490) // Don't use reinterpret_cast
		auto* headerPtr = reinterpret_cast<SimplifiedPtpHeader*>(buffer.data());
		headerPtr->transportSpecificMessageType = static_cast<uint8_t>(PtpMessageType::Delay_Resp);
		headerPtr->sequenceId = SwapEndianness(m_sequenceId);
		return buffer;
	}


}