#ifndef SKREDE_RSI_KUKARSIINTERFACE_H
#define SKREDE_RSI_KUKARSIINTERFACE_H

#include "skrede/rsi/spinlock.h"
#include "skrede/rsi/udpserver.h"
#include "skrede/rsi/kukarsixmlserializer.h"

#include <mutex>
#include <atomic>

namespace skrede::rsi {

class KukaRsiInterface
{
public:
    KukaRsiInterface(const std::string &rsi_host, uint16_t rsi_port, size_t joint_count, std::function<void(const Eigen::ArrayXd &)> joint_pos_recv_callback);

    void setJointPositionsRad(const Eigen::ArrayXd &joint_position_setpoints_rad);

    bool start();
    void stop();

private:
    uint64_t m_ipoc;
    uint16_t m_rsi_port;
    size_t m_joint_count;
    std::string m_rsi_host;
    UDPServer m_rsi_server;
    Spinlock m_state_lock;
    std::atomic<bool> m_running;
    std::atomic<bool> m_rsi_io_active;
    std::atomic<bool> m_initial_position_set;
    std::atomic<bool> m_mirror_remote_setpoint;
    std::chrono::microseconds m_previous_timestamp;
    KukaRsiXmlSerializer m_serializer;
    Eigen::ArrayXd m_joint_setpoint;
    Eigen::ArrayXd m_joint_correction;
    Eigen::ArrayXd m_joint_rsi_setpoint;
    Eigen::ArrayXd m_joint_rsi_inital_pos;
    Eigen::ArrayXd m_joint_measured_position;
    Eigen::Vector3d m_euler_current_orientation;
    Eigen::Vector3d m_cartesian_current_position;
    Eigen::Vector3d m_euler_rsi_setpoint_orientation;
    Eigen::Vector3d m_cartesian_rsi_setpoint_position;
    std::function<void(const Eigen::ArrayXd &)> m_joint_pos_recv_callback;
    std::chrono::system_clock::time_point m_time_previous_message_received;

    void listeningStatusChanged(bool connected);
    void rsiMessageReceived(const std::string &host, uint16_t port, void *payload, size_t length);

    void parseRsiMessage(void *payload, size_t length);

    void setStateParameters(const KukaRsiState &state);

    void sendRsiSetpointMessage(const std::string &host, uint16_t port);

    void unlocked_setJointSetpoints(const Eigen::ArrayXd &setpoints);
};

}

#endif
