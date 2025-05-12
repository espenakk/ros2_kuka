#ifndef SKREDE_RSI_UDPSERVER_H
#define SKREDE_RSI_UDPSERVER_H

#include <string>
#include <thread>
#include <functional>

#include <asio/io_service.hpp>

#include <asio/ip/udp.hpp>

namespace skrede::rsi {

class UDPServer
{
public:
    UDPServer(std::function<void (const std::string &, uint16_t, void *, size_t)> receiver, std::function<void (bool)> socket_open_listener, uint32_t buffer_size = 32768u);
    virtual ~UDPServer();

    uint16_t port() const;
    std::string address() const;

    bool listen();
    bool listen(uint16_t port);
    bool listen(const std::string &host);
    bool listen(const std::string &host, uint16_t port);

    bool is_listening() const;

    bool send(const std::string &host, uint16_t port, const std::string &payload);

    void close();

private:
    std::thread m_worker;
    uint32_t m_buffer_size;
    asio::io_service m_io_service;
    asio::ip::udp::socket m_socket;
    std::unique_ptr<char[]> m_buffer;
    asio::ip::udp::endpoint m_endpoint;
    asio::ip::udp::endpoint m_sender_endpoint;
    std::function<void (bool)> m_socket_open_listener;
    std::function<void(const std::string &host, uint16_t, void *, size_t)> m_receiver;

    void on_receive(const std::error_code &error, size_t bytes_transferred) const;
};

}

#endif
