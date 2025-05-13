#include "rsi/kukarsixmlserializer.h"

#include <sstream>

using namespace rsi;

KukaRsiXmlSerializer::KukaRsiXmlSerializer(size_t joint_count)
    : m_joint_count(joint_count)
{
}

std::string KukaRsiXmlSerializer::serialize(const Eigen::ArrayXd &setpoint, uint64_t ipoc)
{
    assert(m_joint_count == setpoint.size());
    pugi::xml_document doc;
    auto sender_node = doc.append_child("Sen");
    sender_node.append_attribute("Type").set_value("ImFree");
    auto ak_node = sender_node.append_child("AK");
    for(auto i = 0u; i < setpoint.size(); i++)
    {
        auto node_name = "A" + std::to_string(i + 1);
        ak_node.append_attribute(node_name.c_str()).set_value(setpoint[i] * RAD2DEG);
    }
    auto ipoc_node = sender_node.append_child("IPOC");
    ipoc_node.text().set(ipoc);
    std::stringstream ss;
    doc.print(ss, "", pugi::format_indent);
    return ss.str();
}

std::pair<bool, KukaRsiState> KukaRsiXmlSerializer::deserialize(void *payload, size_t length)
{
    pugi::xml_document doc;
    auto result = doc.load_buffer(payload, length);
    if(result.status != pugi::status_ok || length == 0u)
        throw std::runtime_error("Failed to deserialize RSI payload.");
    auto robot = doc.child("Rob");
    if(robot.empty())
        return std::make_pair(false, KukaRsiState(m_joint_count));
    KukaRsiState state(m_joint_count);
    parseJointSetpoints(state, robot.child("ASPos"));
    parseCartesianSetpoints(state, robot.child("RSol"));
    parseActualJointPositions(state, robot.child("AIPos"));
    parseActualCartesianPositions(state, robot.child("RIst"));
    state.ipoc = std::stoull(robot.child_value("IPOC"));
    return std::make_pair(true, state);
}

void KukaRsiXmlSerializer::parseJointSetpoints(KukaRsiState &state, const pugi::xml_node &node) const
{
    for(auto i = 0u; i < m_joint_count; i++)
    {
        std::string node_name = "A" + std::to_string(i + 1);
        state.joint_setpoint[i] = node.attribute(node_name.c_str()).as_double() * DEG2RAD;
    }
}

void KukaRsiXmlSerializer::parseCartesianSetpoints(KukaRsiState &state, const pugi::xml_node &node) const
{
    state.cartesian_position_setpoint[0] = node.attribute("X").as_double();
    state.cartesian_position_setpoint[1] = node.attribute("Y").as_double();
    state.cartesian_position_setpoint[2] = node.attribute("Z").as_double();
    state.cartesian_orientation_setpoint[0] = node.attribute("A").as_double() * DEG2RAD;
    state.cartesian_orientation_setpoint[1] = node.attribute("B").as_double() * DEG2RAD;
    state.cartesian_orientation_setpoint[2] = node.attribute("C").as_double() * DEG2RAD;
}

void KukaRsiXmlSerializer::parseActualJointPositions(KukaRsiState &state, const pugi::xml_node &node) const
{
    for(auto i = 0u; i < m_joint_count; i++)
    {
        auto node_name = "A" + std::to_string(i + 1);
        state.joint_measurement[i] = node.attribute(node_name.c_str()).as_double() * DEG2RAD;
    }
}

void KukaRsiXmlSerializer::parseActualCartesianPositions(KukaRsiState &state, const pugi::xml_node &node) const
{
    state.cartesian_position_measurement[0] = node.attribute("X").as_double();
    state.cartesian_position_measurement[1] = node.attribute("Y").as_double();
    state.cartesian_position_measurement[2] = node.attribute("Z").as_double();
    state.cartesian_orientation_measurement[0] = node.attribute("A").as_double() * DEG2RAD;
    state.cartesian_orientation_measurement[1] = node.attribute("B").as_double() * DEG2RAD;
    state.cartesian_orientation_measurement[2] = node.attribute("C").as_double() * DEG2RAD;
}
