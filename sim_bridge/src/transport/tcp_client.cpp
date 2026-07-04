#include "sim_bridge/transport/tcp_client.hpp"

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

TcpClient::TcpClient() {
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
    const std::string payload = line + "\n";
    const char* cursor = payload.c_str();
    int remaining = static_cast<int>(payload.size());
    while (remaining > 0) {
        const int sent = static_cast<int>(::send(socket_, cursor, remaining, 0));
        if (sent <= 0) {
            throw std::runtime_error("socket send failed");
        }
        cursor += sent;
        remaining -= sent;
    }
}

bool TcpClient::read_line(std::string& line) {
    line.clear();
    char ch = '\0';
    while (true) {
        const int received = static_cast<int>(::recv(socket_, &ch, 1, 0));
        if (received == 0) {
            return !line.empty();
        }
        if (received < 0) {
            throw std::runtime_error("socket receive failed");
        }
        if (ch == '\n') {
            return true;
        }
        line.push_back(ch);
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
