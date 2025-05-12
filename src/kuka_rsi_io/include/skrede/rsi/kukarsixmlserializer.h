#ifndef SKREDE_RSI_KUKARSIXMLSERIALIZER_H
#define SKREDE_RSI_KUKARSIXMLSERIALIZER_H

#include <Eigen/Dense>

#include <pugixml.hpp>

#define DEG2RAD 0.01745329252
#define RAD2DEG 57.295779513

namespace skrede::rsi {

struct KukaRsiState
{
    KukaRsiState(size_t joint_count)
        : ipoc(0u)
        , joint_setpoint(joint_count)
        , joint_measurement(joint_count)
    {

    }

    uint64_t ipoc;
    Eigen::ArrayXd joint_setpoint;
    Eigen::ArrayXd joint_measurement;
    Eigen::Vector3d cartesian_position_setpoint;
    Eigen::Vector3d cartesian_orientation_setpoint;
    Eigen::Vector3d cartesian_position_measurement;
    Eigen::Vector3d cartesian_orientation_measurement;
};

class KukaRsiXmlSerializer
{
public:
    KukaRsiXmlSerializer(size_t joint_count);

    std::string serialize(const Eigen::ArrayXd &setpoint, uint64_t ipoc);
    std::pair<bool, KukaRsiState> deserialize(void *payload, size_t length);

private:
    size_t m_joint_count;
    void parseJointSetpoints(KukaRsiState &state, const pugi::xml_node &node) const;
    void parseCartesianSetpoints(KukaRsiState &state, const pugi::xml_node &node) const;
    void parseActualJointPositions(KukaRsiState &state, const pugi::xml_node &node) const;
    void parseActualCartesianPositions(KukaRsiState &state, const pugi::xml_node &node) const;
};

}

#endif
