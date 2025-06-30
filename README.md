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

## Kalman Filter Insights (1-D)

- **Adaptive process–noise `Q`**  
  *Q* is recomputed from the _squared_ step size of the estimate  
  (`Δestimate²`) and then clamped to **[1 e-6 … 10]**.  
  → The filter *loosens* when path-delay drifts and *tightens* when it is stable.

- **Adaptive measurement–noise `R`**  
  Uses a 20-sample sliding window of raw delays, computes the unbiased variance,  
  and floors it at **1 e-6** to prevent divide-by-zero.

- **Automatic freeze-protection**  
  Before every gain calculation the code inflates `P` by ×10 whenever  
  `P < 0.1 · R`.  
  This guarantees a minimum Kalman-gain of **≈ 0.09** so the filter never locks up when `R` explodes.

- **Consistent-tracking diagnostics**  
  Keeps bounded histories of **innovation** and **NIS** (χ² consistency):  
  * mean innovation → bias check (should hover near 0)  
  * mean NIS → tuning check (should hover near 1) — printed each cycle.

- **One-line gain formula**  

  ```math
  K \;=\; \frac{P/R}{1 + P/R}

## 📝 TODO

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