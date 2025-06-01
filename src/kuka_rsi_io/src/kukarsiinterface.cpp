#include "rsi/kukarsiinterface.h"

#include <iostream>

#include <spdlog/spdlog.h>

using namespace rsi;

KukaRsiInterface::KukaRsiInterface(const std::string &rsi_host, uint16_t rsi_port, size_t joint_count, std::function<void(const Eigen::ArrayXd &)> joint_pos_recv_callback)
    : m_ipoc(0u)
    , m_rsi_port(rsi_port)
    , m_joint_count(joint_count)
    , m_rsi_host(rsi_host)
    , m_rsi_server(std::bind(&KukaRsiInterface::rsiMessageReceived, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
          std::bind(&KukaRsiInterface::listeningStatusChanged, this, std::placeholders::_1))
    , m_running{false}
    , m_initial_position_set{false}
    , m_mirror_remote_setpoint{true}
    , m_serializer(joint_count)
    , m_joint_setpoint(Eigen::ArrayXd::Zero(joint_count))
    , m_joint_correction(Eigen::ArrayXd::Zero(joint_count))
    , m_joint_rsi_setpoint(Eigen::ArrayXd::Zero(joint_count))
    , m_joint_rsi_inital_pos(Eigen::ArrayXd::Zero(joint_count))
    , m_joint_pos_recv_callback(std::move(joint_pos_recv_callback))
    , m_joint_measured_position(Eigen::ArrayXd::Zero(joint_count))
{
}

void KukaRsiInterface::setJointPositionsRad(const Eigen::ArrayXd &joint_position_setpoints_rad)
{
    {
        std::lock_guard<Spinlock> l(m_state_lock);
        unlocked_setJointSetpoints(joint_position_setpoints_rad);
    }
    m_mirror_remote_setpoint.store(false);
}

bool KukaRsiInterface::start()
{
    if(m_running.load())
        return false;
    m_mirror_remote_setpoint.store(true);
    m_rsi_io_active.store(false);
    m_time_previous_message_received = std::chrono::system_clock::now();
    return m_rsi_server.listen(m_rsi_host, m_rsi_port);
}

void KukaRsiInterface::stop()
{
    m_rsi_server.close();
}

void KukaRsiInterface::listeningStatusChanged(bool connected)
{
    m_running = connected;
}

void KukaRsiInterface::rsiMessageReceived(const std::string &host, uint16_t port, void *payload, size_t length)
{
    auto now = std::chrono::system_clock::now();
    bool measure_duration = false;
    if(m_rsi_io_active == false)
    {
        spdlog::warn("Started new RSI connection");
        m_initial_position_set.store(false);
        m_mirror_remote_setpoint.store(true);
        m_rsi_io_active.store(true);
        m_time_previous_message_received = std::chrono::system_clock::now();
    }
    else
        measure_duration = true;
    if(measure_duration)
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - m_time_previous_message_received);
        if(duration.count() > 100000)
        {
            m_mirror_remote_setpoint.store(true);
            m_rsi_io_active.store(false);
            spdlog::error("IPOC: %u - RSI RTT = %uus > 4000us, aborting previous connection", m_ipoc + 1u, duration.count());
            return;
        }
    }
//     spdlog::info("Parsing RSI message");
    parseRsiMessage(payload, length);
//     spdlog::info("Sending RSI response");
    sendRsiSetpointMessage(host, port);
    if(measure_duration)
        m_time_previous_message_received = now;
}

void KukaRsiInterface::parseRsiMessage(void *payload, size_t length)
{
    std::lock_guard<Spinlock> l(m_state_lock);
    auto result = m_serializer.deserialize(payload, length);
    if(!result.first)
    {
        spdlog::error("Ignored invalid RSI message");
        return;
    }
    setStateParameters(result.second);
    if(m_mirror_remote_setpoint.load())
    {
        m_joint_setpoint = m_joint_measured_position;
        m_joint_correction = m_joint_setpoint - m_joint_rsi_inital_pos;
    }
    if(m_initial_position_set.load() == false)
    {
        m_joint_rsi_inital_pos = m_joint_measured_position;
        m_joint_setpoint = m_joint_measured_position;
        m_joint_correction = Eigen::VectorXd::Zero(m_joint_count);
        m_initial_position_set.store(true);
    }
}

void KukaRsiInterface::setStateParameters(const KukaRsiState &rsi_state)
{
    m_ipoc = rsi_state.ipoc;
    m_joint_rsi_setpoint = rsi_state.joint_setpoint;
    m_joint_measured_position = rsi_state.joint_measurement;
    m_euler_current_orientation = rsi_state.cartesian_orientation_measurement;
    m_cartesian_current_position = rsi_state.cartesian_position_measurement;
    m_euler_rsi_setpoint_orientation = rsi_state.cartesian_orientation_setpoint;
    m_cartesian_rsi_setpoint_position = rsi_state.cartesian_position_setpoint;
    m_joint_pos_recv_callback(m_joint_measured_position);
}

void KukaRsiInterface::sendRsiSetpointMessage(const std::string &host, uint16_t port)
{
    auto payload = m_serializer.serialize(m_joint_correction, m_ipoc);
    m_rsi_server.send(host, port, payload);
}

void KukaRsiInterface::unlocked_setJointSetpoints(const Eigen::ArrayXd &setpoints)
{
    m_joint_setpoint = setpoints;
    if(m_initial_position_set.load())
        m_joint_correction = (setpoints - m_joint_rsi_inital_pos);
    // Debug
    // std::stringstream ss;
    // ss << "Set joint positions: " << m_joint_correction.transpose() << std::endl;
    // spdlog::info(ss.str());
}
