

#include "Utils.h"

#include <boost/asio.hpp>

namespace PTP
{
	class Server
	{
	public:

		Server(boost::asio::io_context& ioContext
			, const std::string& ipAddress
			, unsigned short eventPort
			, unsigned short generalPort);

		Server(const Server&) = delete;
		Server& operator=(const Server&) = delete;
		Server(Server&&) = delete;
		Server& operator=(Server&&) = delete;


	private:

        boost::asio::awaitable<void> Broadcast();
		boost::asio::awaitable<void> Receive();
		boost::asio::awaitable<void> SendSyncMessage();
		boost::asio::awaitable<void> SendFollowUpMessage();
		std::vector<uint8_t> CreateSyncMessage();
		std::vector<uint8_t> CreateFollowUpMessage();
		boost::asio::awaitable<void>  SendDelayResponse(
			PtpTimestamp requestTimeStamp,
			std::vector<uint8_t> buffer, boost::asio::ip::udp::endpoint endpoint);
		std::vector<uint8_t> CreateDelayResponseMessage(
			PtpTimestamp requestTimeStamp,
			std::vector<uint8_t> receiveBuffer);

		boost::asio::io_context& m_ioContext;
		boost::asio::ip::address m_localAdapter;
		boost::asio::ip::udp::socket m_eventSocket;
		boost::asio::ip::udp::socket m_generalSocket;
		boost::asio::ip::udp::endpoint m_remoteEventEndpoint;
		boost::asio::ip::udp::endpoint m_remoteGeneralEndpoint;
		uint16_t m_sequenceId{ 0 };
		PtpTimestamp m_syncTimestamp;
		PtpTimestamp m_requestTimeStamp;
	};
}