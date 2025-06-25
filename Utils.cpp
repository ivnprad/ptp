#include "Utils.h"

namespace PTP
{
	void RethrowException(std::exception_ptr eptr)
	{
		if (eptr) {
			std::rethrow_exception(eptr);
		}
	}

	boost::asio::awaitable<void> WaitForTimeout(std::chrono::milliseconds duration)
	{
		boost::asio::steady_timer timer(co_await boost::asio::this_coro::executor);
		timer.expires_after(duration);
		co_await timer.async_wait(boost::asio::use_awaitable);
	}
	
	PtpTimestamp GetCurrentPtpTime()
	{

		using namespace std::chrono;

		const auto now = high_resolution_clock::now();
		const auto duration = now.time_since_epoch();

		const auto secs{ duration_cast<seconds>(duration).count() };
		const auto nanos{ duration_cast<nanoseconds>(duration).count() % 1000000000 };

		return { SwapEndianness(static_cast<uint32_t>(secs)),
			SwapEndianness(static_cast<uint32_t>(nanos)) };
	}

	
}