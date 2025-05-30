#include "rsi/udpserver.h"

#include <spdlog/spdlog.h>

using namespace rsi;

UDPServer::UDPServer(std::function<void(const std::string &, uint16_t, void *, size_t)> receiver, std::function<void(bool)> socket_open_listener, uint32_t buffer_size)
    : m_buffer_size(buffer_size)
    , m_socket(m_io_service)
    , m_buffer(std::make_unique<char[]>(buffer_size))
    , m_socket_open_listener(socket_open_listener)
    , m_receiver(receiver)
{
}

UDPServer::~UDPServer()
{
    if(is_listening())
        close();
    if(m_worker.joinable())
        m_worker.join();
}

uint16_t UDPServer::port() const
{
    return m_socket.local_endpoint().port();
}

std::string UDPServer::address() const
{
    return m_socket.local_endpoint().address().to_string();
}

bool UDPServer::listen()
{
    return listen(0u);
}

bool UDPServer::listen(const std::string &host)
{
    return listen(host, 0u);
}

bool UDPServer::listen(uint16_t port)
{
    return listen(std::string(), port);
}

bool UDPServer::listen(const std::string &host, uint16_t port)
{
    if(is_listening())
        return false;
    if(host.empty())
        m_endpoint = asio::ip::udp::endpoint(asio::ip::udp::v4(), port);
    else {
        spdlog::info("Debug listen {} -> {}", host, asio::ip::address::from_string(host).to_string());
        m_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(host), port);
    }
    m_socket.open(asio::ip::udp::v4());
    m_socket.set_option(asio::ip::udp::socket::reuse_address(true));
    m_socket.bind(m_endpoint);
    bool ok = m_socket.is_open();
    if(ok)
    {
        m_worker = std::thread([&]()
        {
            spdlog::info("UDP Read thread started");

            do {
                m_io_service.run();
                spdlog::warn("IO Service stopped");
                m_io_service.reset();
                m_socket_open_listener(true);
                m_socket.async_receive_from(asio::buffer(m_buffer.get(), m_buffer_size), m_sender_endpoint, std::bind(&UDPServer::on_receive, this, std::placeholders::_1, std::placeholders::_2));
            } while(m_socket.is_open());


            spdlog::info("UDP Read thread stopped");
        });
        spdlog::info("Started udp server on {}:{}", address(), this->port());
        m_socket_open_listener(true);
        m_socket.async_receive_from(asio::buffer(m_buffer.get(), m_buffer_size), m_sender_endpoint, std::bind(&UDPServer::on_receive, this, std::placeholders::_1, std::placeholders::_2));
    }
    return ok;
}

bool UDPServer::is_listening() const
{
    return m_socket.is_open();
}

bool UDPServer::send(const std::string &host, uint16_t port, const std::string &payload)
{
    if(is_listening()) {
        try
        {
            auto payload_size = payload.size();
            auto sent_size = m_socket.send_to(asio::buffer(payload, payload_size), asio::ip::udp::endpoint(asio::ip::address::from_string(host), port));
            spdlog::info("Sending UDP Message pl{} snt{}", payload_size, sent_size);

            return true;
        }
        catch(const std::exception &e)
        {
            spdlog::warn("Unable to send udp message to {}:{}. {}", host, port, e.what());
        }
        spdlog::warn("Problem sending UDP Message");
    }

    return false;
}

void UDPServer::close()
{
    if(m_socket.is_open())
    {
        spdlog::info("Closing UDP server {}:{}", address(), port());
        m_socket.close();
        m_io_service.stop();
        if(m_worker.joinable())
            m_worker.join();
        m_socket_open_listener(false);
    }
    spdlog::info("!!!!!!!!Closed UDP server {}:{}", address(), port());
}

void UDPServer::on_receive(const std::error_code &error, size_t bytes_transferred) const
{
    // spdlog::info("Got Something");
    if(!error || error == asio::error::message_size)
        m_receiver(m_sender_endpoint.address().to_string(), m_sender_endpoint.port(), m_buffer.get(), bytes_transferred);       
}
