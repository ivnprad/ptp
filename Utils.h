
#pragma once

#include <boost/asio.hpp>
#include <bit>

namespace PTP
{
	void RethrowException(std::exception_ptr eptr); 
	boost::asio::awaitable<void> WaitForTimeout(std::chrono::milliseconds duration);

	constexpr inline auto c_lower4BitsMask{ 0x0F };
	constexpr inline auto c_ptpEventPort{ 1319/*319*/ };
	constexpr inline auto c_ptpGeneralPort{ 1320/*320*/ };
	constexpr inline auto c_serverIP{ "127.0.0.10"};
	constexpr inline auto c_clientIP{ "127.0.0.1"};

	constexpr inline auto c_brodcastTimeout{ std::chrono::milliseconds(250) };
	constexpr inline auto c_delayRequestTimeout{ std::chrono::seconds(2) };
	constexpr inline auto c_cleanupInterval = std::chrono::seconds(5);
	constexpr inline auto c_entryStaleTimeout = std::chrono::seconds(4); // An entry is stale if older than this.
	constexpr inline size_t c_maxTimestampSets = 20;

	const inline boost::asio::ip::address_v4 c_multicastEvent{ { 224, 0, 1, 129 } };
	const inline boost::asio::ip::address_v4 c_multicastGeneral{ { 224, 0, 1, 130 } };

	enum class PtpMessageType : uint8_t
	{
		Sync = 0x0,
		Delay_Req = 0x1,
		Pdelay_Req = 0x2,
		Pdelay_Resp = 0x3,
		Follow_Up = 0x8,
		Delay_Resp = 0x9,
		Pdelay_Resp_Follow_Up = 0xA,
		Announce = 0xB,
		Signaling = 0xC,
		Management = 0xD,
		Unknown = 0xFF
	};

	template <std::integral T>
	[[nodiscard]]
	constexpr T SwapEndianness(T value)
	{
		if constexpr (std::endian::native == std::endian::little)
		{
			return std::byteswap(value);
		}
		return value;
	}

	// NOTE: Real PTP headers are more complex and require proper handling of endianness and field layouts.
	#pragma pack(push, 1) // Ensure no padding is added between members
	struct SimplifiedPtpHeader
	{
		uint8_t transportSpecificMessageType; // Upper 4 bits: transportSpecific, lower 4 bits: messageType
		uint8_t versionPtp;                     // Upper 4 bits: reserved, lower 4 bits: PTP version
		uint16_t messageLength;                 // Total message length in bytes (big-endian in real PTP)
		uint8_t domainNumber;                   // PTP domain (default is 0)
		uint8_t reserved1;                       // Reserved or SdoId (implementation specific)
		uint16_t flags;                          // Flags (e.g., two-step, unicast)
		int64_t correctionField;                // Time correction in nanoseconds
		uint32_t reserved2;                      // Reserved (often 0)
		uint8_t sourcePortIdentity[10];        // 8 bytes clockIdentity + 2 bytes portNumber
		uint16_t sequenceId;                    // Sequence number of message
		uint8_t controlField;                   // Used in older PTP for messageType class (deprecated)
		int8_t logMessageInterval;             // log2(interval) between messages

		PtpMessageType GetMessageType() const
		{
			const auto messageType = transportSpecificMessageType & c_lower4BitsMask; // Upper 4 bits
			switch (messageType)
			{
				case 0x0: return PtpMessageType::Sync;
				case 0x1: return PtpMessageType::Delay_Req;
				case 0x2: return PtpMessageType::Pdelay_Req;
				case 0x3: return PtpMessageType::Pdelay_Resp;
				case 0x8: return PtpMessageType::Follow_Up;
				case 0x9: return PtpMessageType::Delay_Resp;
				case 0xA: return PtpMessageType::Pdelay_Resp_Follow_Up;
				case 0xB: return PtpMessageType::Announce;
				case 0xC: return PtpMessageType::Signaling;
				case 0xD: return PtpMessageType::Management;
				default:  return PtpMessageType::Unknown;
			}
		}

		uint8_t GetPtpVersion() const
		{
			return versionPtp & c_lower4BitsMask;
		}
	};

	//The PTP Timestamp format is actually: 48 - bit seconds(6 bytes) 32 - bit nanoseconds(4 bytes)
	struct PtpTimestamp
	{
		uint32_t seconds;
		uint32_t nanoseconds;

		int64_t to_nanoseconds() const
		{
			return static_cast<int64_t>(SwapEndianness(seconds)) * 1000000000LL +
				static_cast<int64_t>(SwapEndianness(nanoseconds));
		}
	};
	#pragma pack(pop)

	PtpTimestamp GetCurrentPtpTime();

}