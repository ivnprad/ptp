

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
		boost::asio::awaitable<void> WaitDelayRequest();
		boost::asio::awaitable<void> SendRequestReponse();
		std::vector<uint8_t> CreateSyncMessage();
		std::vector<uint8_t> CreateFollowUpMessage();
		std::vector<uint8_t> CreateDelayResponseMessage();

		boost::asio::io_context& m_ioContext;
		boost::asio::ip::address m_localAdapter;
		boost::asio::ip::udp::socket m_eventSocket;
		boost::asio::ip::udp::socket m_generalSocket;
		boost::asio::ip::udp::endpoint m_remoteEventEndpoint;
		boost::asio::ip::udp::endpoint m_remoteGeneralEndpoint;
		std::array<char, 1024> m_eventRecvBuffer{ {} };
		std::array<char, 1024> m_generalRecvBuffer{ {} };
		uint16_t m_sequenceId{ 0 };
		PtpTimestamp m_syncTimestamp;
		PtpTimestamp m_requestTimeStamp;
	};
}