#include "modbus_slave/Slave.hpp"

Slave::Slave()
	: Node("modbus_slave_node"), m_server(1502),
	m_programIndex(0), m_param1(0.f), m_param2(0),
	m_param3(0), m_param4(0)
{
}

void Slave::update()
{
	m_server.update();
	uint16_t reg0 = m_server.readHoldingRegisterInt(0);
	// Retrieve bits from word
	bool start = reg0 & 1;
	bool stop = reg0 & (1 << 1);
	bool visuMode = reg0 & (1 << 2);

	uint16_t inReg0 = 0;
	// Set ready if ready
	inReg0 |= 1 << 3;
	// Set running if running
	inReg0 |= 1 << 4;
	m_server.writeInputRegister(0, inReg0);

	// Get selected program index
	m_programIndex = m_server.readHoldingRegisterInt(1);
	// Write current program index
	m_server.writeInputRegister(5, m_programIndex);

	// Update params
	m_param1 = m_server.readHoldingRegisterFloat(6);
	m_param2 = m_server.readHoldingRegisterInt(2);
	m_param3 = m_server.readHoldingRegisterInt(3);
	m_param4 = m_server.readHoldingRegisterInt(4);
}
