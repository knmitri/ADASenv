#include "sim_bridge/transport/tcp_client.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sim_bridge {

TcpClient::TcpClient(std::size_t max_frame_size)
    : max_frame_size_(max_frame_size) {
    if (max_frame_size_ == 0) {
        throw std::invalid_argument("maximum TCP frame size must be greater than zero");
    }
#ifdef _WIN32
    WSADATA data;
    const int rc = WSAStartup(MAKEWORD(2, 2), &data);
    if (rc != 0) {
        throw std::runtime_error("WSAStartup failed");
    }
#endif
}

TcpClient::~TcpClient() {
    close_socket();
#ifdef _WIN32
    WSACleanup();
#endif
}

void TcpClient::connect_to(const std::string& host, const std::string& port) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &result);
    if (rc != 0) {
        throw std::runtime_error("getaddrinfo failed for " + host + ":" + port);
    }

    for (addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
        socket_ = ::socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (socket_ == kInvalidSocket) {
            continue;
        }
        if (::connect(socket_, ptr->ai_addr, static_cast<int>(ptr->ai_addrlen)) == 0) {
            freeaddrinfo(result);
            return;
        }
        close_socket();
    }
    freeaddrinfo(result);
    throw std::runtime_error("could not connect to " + host + ":" + port);
}

void TcpClient::write_line(const std::string& line) {
    if (line.size() > max_frame_size_) {
        throw std::length_error("outgoing TCP frame exceeds configured maximum size");
    }
    if (line.find('\n') != std::string::npos) {
        throw std::invalid_argument("TCP frame must not contain an unescaped newline");
    }
    const std::string payload = line + "\n";
    const char* cursor = payload.c_str();
    std::size_t remaining = payload.size();
    while (remaining > 0) {
        const auto chunk_size = std::min<std::size_t>(remaining, 4096);
        const int sent = static_cast<int>(
            ::send(socket_, cursor, static_cast<int>(chunk_size), 0)
        );
        if (sent <= 0) {
            throw std::runtime_error("socket send failed");
        }
        cursor += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
}

bool TcpClient::read_line(std::string& line) {
    line.clear();
    while (true) {
        const std::size_t delimiter = read_buffer_.find('\n');
        if (delimiter != std::string::npos) {
            if (delimiter > max_frame_size_) {
                throw std::length_error("incoming TCP frame exceeds configured maximum size");
            }
            line.assign(read_buffer_, 0, delimiter);
            read_buffer_.erase(0, delimiter + 1);
            return true;
        }
        if (read_buffer_.size() > max_frame_size_) {
            throw std::length_error("incoming TCP frame exceeds configured maximum size");
        }

        std::array<char, 4096> buffer{};
        const int received = static_cast<int>(::recv(
            socket_,
            buffer.data(),
            static_cast<int>(buffer.size()),
            0
        ));
        if (received == 0) {
            if (read_buffer_.empty()) {
                return false;
            }
            throw std::runtime_error("connection closed before TCP frame was complete");
        }
        if (received < 0) {
            throw std::runtime_error("socket receive failed");
        }
        read_buffer_.append(buffer.data(), static_cast<std::size_t>(received));
    }
}

void TcpClient::close_socket() {
    if (socket_ == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket_);
#else
    close(socket_);
#endif
    socket_ = kInvalidSocket;
}

}  // namespace sim_bridge
