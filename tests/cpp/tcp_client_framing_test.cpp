#include <algorithm>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "sim_bridge/transport/tcp_client.hpp"

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

using sim_bridge::SocketHandle;

void close_socket(SocketHandle socket) {
    if (socket == sim_bridge::kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

class SocketRuntime {
public:
    SocketRuntime() {
#ifdef _WIN32
        WSADATA data;
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("test WSAStartup failed");
        }
#endif
    }

    ~SocketRuntime() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

class LocalServer {
public:
    explicit LocalServer(std::string payload) : payload_(std::move(payload)) {
        listener_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listener_ == sim_bridge::kInvalidSocket) {
            throw std::runtime_error("could not create test server socket");
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            close_socket(listener_);
            throw std::runtime_error("could not bind test server socket");
        }
        if (::listen(listener_, 1) != 0) {
            close_socket(listener_);
            throw std::runtime_error("could not listen on test server socket");
        }

#ifdef _WIN32
        int address_size = static_cast<int>(sizeof(address));
#else
        socklen_t address_size = static_cast<socklen_t>(sizeof(address));
#endif
        if (::getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
            close_socket(listener_);
            throw std::runtime_error("could not inspect test server socket");
        }
        port_ = ntohs(address.sin_port);
        thread_ = std::thread([this] { serve(); });
    }

    ~LocalServer() {
        if (thread_.joinable()) {
            thread_.join();
        }
        close_socket(listener_);
    }

    std::string port() const {
        return std::to_string(port_);
    }

    void join() {
        if (thread_.joinable()) {
            thread_.join();
        }
        if (!server_ok_) {
            throw std::runtime_error("test TCP server failed to send its frame");
        }
    }

private:
    void serve() {
        const SocketHandle peer = ::accept(listener_, nullptr, nullptr);
        if (peer == sim_bridge::kInvalidSocket) {
            return;
        }

        std::size_t offset = 0;
        while (offset < payload_.size()) {
            const std::size_t chunk_size = std::min<std::size_t>(payload_.size() - offset, 3);
            const int sent = static_cast<int>(::send(
                peer,
                payload_.data() + offset,
                static_cast<int>(chunk_size),
                0
            ));
            if (sent <= 0) {
                close_socket(peer);
                return;
            }
            offset += static_cast<std::size_t>(sent);
        }
        server_ok_ = true;
        close_socket(peer);
    }

    std::string payload_;
    SocketHandle listener_ = sim_bridge::kInvalidSocket;
    unsigned short port_ = 0;
    std::thread thread_;
    bool server_ok_ = false;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Function>
void require_throws(Function function, const std::string& expected, const std::string& context) {
    try {
        function();
    } catch (const Exception& error) {
        require(std::string(error.what()).find(expected) != std::string::npos, context);
        return;
    }
    throw std::runtime_error(context + ": no exception was thrown");
}

void test_chunked_and_buffered_frames() {
    LocalServer server("{\"type\":\"hello\"}\n{\"type\":\"ack\"}\n");
    sim_bridge::TcpClient client(64);
    client.connect_to("127.0.0.1", server.port());

    std::string line;
    require(client.read_line(line) && line == R"({"type":"hello"})",
            "chunked first frame was not reconstructed");
    require(client.read_line(line) && line == R"({"type":"ack"})",
            "buffered second frame was not returned");
    require(!client.read_line(line), "closed connection should report no further complete frame");
    server.join();
}

void test_oversized_frame_is_rejected() {
    LocalServer server("12345\n");
    sim_bridge::TcpClient client(4);
    client.connect_to("127.0.0.1", server.port());
    std::string line;
    require_throws<std::length_error>(
        [&] { client.read_line(line); },
        "exceeds configured maximum",
        "oversized TCP frame was not rejected"
    );
    server.join();
}

void test_outgoing_frame_limits() {
    sim_bridge::TcpClient client(4);
    require_throws<std::length_error>(
        [&] { client.write_line("12345"); },
        "exceeds configured maximum",
        "oversized outgoing TCP frame was not rejected"
    );
    sim_bridge::TcpClient multiline_client(16);
    require_throws<std::invalid_argument>(
        [&] { multiline_client.write_line("one\ntwo"); },
        "must not contain an unescaped newline",
        "multi-line TCP frame was not rejected"
    );
}

void test_incomplete_frame_and_disconnect() {
    {
        LocalServer server("{\"type\":\"ack\"}");
        sim_bridge::TcpClient client;
        client.connect_to("127.0.0.1", server.port());
        std::string line;
        require_throws<std::runtime_error>(
            [&] { client.read_line(line); },
            "before TCP frame was complete",
            "incomplete frame was not reported"
        );
        server.join();
    }
    {
        LocalServer server("");
        sim_bridge::TcpClient client;
        client.connect_to("127.0.0.1", server.port());
        std::string line;
        require(!client.read_line(line), "empty disconnect should report end of stream");
        server.join();
    }
}

}  // namespace

int main() {
    try {
        SocketRuntime runtime;
        test_chunked_and_buffered_frames();
        test_oversized_frame_is_rejected();
        test_outgoing_frame_limits();
        test_incomplete_frame_and_disconnect();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
