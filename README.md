# Precision Time Protocol (PTP) - Modern C++ Implementation

This project implements a simplified **Precision Time Protocol (PTP)** using **Boost Asio coroutines** in modern C++ (C++23). The system consists of a PTP server and client that communicate using multicast/unicast UDP messages and synchronize using a 1D Kalman Filter to smooth the delay measurements.

---

## ✨ Features

- 🧭 PTP Clock Sync using SYNC, FOLLOW_UP, DELAY_REQ, and DELAY_RESP messages
- ⚙️ Boost Asio coroutines for non-blocking asynchronous I/O
- 📶 Multicast and Unicast UDP support
- 📉 1D Kalman filter for delay smoothing and noise adaptation
- 🧪 Built-in diagnostics with NIS (Normalized Innovation Squared) tracking

---

## 🧩 Project Structure

- Main.cpp # CLI entry point
- PtpClient.{h,cpp} # PTP client implementation
- PtpServer.{h,cpp} # PTP server implementation
- KalmanFilter1D.{h,cpp} # Kalman filter for delay smoothing
- Utils.{h,cpp} # Common utilities, timers, timestamp formatting
- README.md # This file

---

## How it works

- Server sends periodic SYNC and FOLLOW_UP messages with timestamps.
- Client listens, captures timestamps, and sends DELAY_REQ.
- Server replies with DELAY_RESP.
- The client uses all timestamps to compute mean path delay and applies a Kalman filter for better stability.
- System logs estimation quality via NIS values, which help identify filter performance or potential issues.

---

## Kalman Filter Insights

- Adjusts process noise (Q) based on estimate changes
- Adjusts measurement noise (R) using recent variance
- Logs diagnostic info like Kalman gain, noise levels, and estimation accuracy

## 📝 TODO

- [ ] **Implement per-request coroutine handler pattern in server**

    > One perpetual Listener task:  
    > Its only job is to `co_await` a new request.  
    > Many temporary Handler tasks:  
    > When the Listener gets a request, it doesn't process it directly.  
    > Instead, it immediately spawns a new, temporary coroutine to handle that one request.  
    > It passes all the necessary information (like the client's endpoint) as parameters to this new handler task.  
    > The Listener immediately loops back to `co_await` the next request, while the handler task for the first client runs concurrently.  
    > This way, the server can accept a new request from Client B while it's still processing Client A, making it truly concurrent and scalable.

- [ ] **Refactor `WaitDelayRequest` to extract `sequenceId` and pass it**

    > The `sequenceId` should be extracted when a request is received.  
    > It must then be passed to `SendRequestReponse()` and `CreateDelayResponseMessage()`.  
    > The `m_sequenceId` member is owned by the broadcast logic and should not be modified elsewhere.

- [ ] Add unit tests for Kalman filter  
- [ ] Extend support to real Ethernet PTP hardware  
- [ ] Add configuration for multicast groups and interface selection

## 🚀 Getting Started

### 🔧 Prerequisites

- C++23 compatible compiler (`clang++`, `g++-13`, MSVC)
- Boost libraries (especially `boost_system`, `boost_program_options`, `boost_asio`)
- `range-v3` (for PTP client filtering & transforms)

---

## Usage
- Start Server ./PTP
- Start Client ./PTP --Client --IpAddress 127.0.0.10

### 🛠️ Compilation (Example: Clang)

```bash
/opt/homebrew/opt/llvm/bin/clang++ -std=gnu++23 -g \
  -fcolor-diagnostics -fansi-escape-codes -pthread \
  -I/opt/homebrew/include -L/opt/homebrew/lib \
  -lboost_system -lboost_program_options \
  Main.cpp PtpClient.cpp PtpServer.cpp Utils.cpp KalmanFilter1D.cpp \
  -o PTP 
```