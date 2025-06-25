# ptp
precision time protocol using modern c++ boost asio coroutines 

Compile Command Clang:
/opt/homebrew/opt/llvm/bin/clang++ -std=gnu++23 -g -fcolor-diagnostics -fansi-escape-codes -pthread -I/opt/homebrew/include -L/opt/homebrew/lib -lboost_system -lboost_program_options Main.cpp PtpClient.cpp PtpServer.cpp Utils.cpp -o PTP 