#pragma once

#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
namespace sim_bridge {
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
}
#else
namespace sim_bridge {
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
}
#endif

namespace sim_bridge {

class TcpClient {
public:
    TcpClient();
    ~TcpClient();

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    void connect_to(const std::string& host, const std::string& port);
    void write_line(const std::string& line);
    bool read_line(std::string& line);

private:
    SocketHandle socket_ = kInvalidSocket;

    void close_socket();
};

}  // namespace sim_bridge
