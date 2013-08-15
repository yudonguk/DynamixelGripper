#ifndef __DYNAMIXEL_GROUP_H__
#define __DYNAMIXEL_GROUP_H__

#include <vector>
#include <device/UART.h>

#include "DynamixelUART.h"

class DynamixelGroup
{
private:
	static const size_t MAX_PAYLOAD_SIZE = 143 - 7;

public:
	DynamixelGroup(Uart* uart_ = NULL);
	~DynamixelGroup();

public:
	void SetUart(Uart* uart_);
	inline void Add(DynamixelUART* pDynamixel) { dynamixelVector.push_back(pDynamixel); }
	inline size_t CountDynamixel() { return dynamixelVector.size(); }
	void Clear();

	size_t SetGoalPosition(const vector<unsigned short>& goalPosition);
	size_t SetTorqueEnable(bool isEnabled);

	size_t GetPresentPosition(std::vector<unsigned short>& currentPosition);

	inline DynamixelUART& operator[](size_t index) { return *dynamixelVector[index]; }

private:
	DynamixelUART broadcastDynamixel;
	std::vector<DynamixelUART*> dynamixelVector;
	Uart* uart;
};

#endif