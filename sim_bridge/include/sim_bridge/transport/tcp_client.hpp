#pragma once

#include <cstddef>
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
    static constexpr std::size_t kDefaultMaxFrameSize = 1024 * 1024;

    explicit TcpClient(std::size_t max_frame_size = kDefaultMaxFrameSize);
    ~TcpClient();

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    void connect_to(const std::string& host, const std::string& port);
    void write_line(const std::string& line);
    bool read_line(std::string& line);

private:
    SocketHandle socket_ = kInvalidSocket;
    const std::size_t max_frame_size_;
    std::string read_buffer_;

    void close_socket();
};

}  // namespace sim_bridge
