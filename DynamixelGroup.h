#ifndef __DYNAMIXEL_GROUP_H__
#define __DYNAMIXEL_GROUP_H__

#include <vector>

#include <boost/shared_ptr.hpp>

#include <device/UART.h>

#include "DynamixelUART.h"

class DynamixelGroup : public std::vector<boost::shared_ptr<DynamixelUART>>
{
private:
	static const size_t MAX_PAYLOAD_SIZE = 143 - 7;

public:
	DynamixelGroup(Uart* uart_ = NULL);
	~DynamixelGroup();

public:
	void SetUart(Uart* uart_);
	
	size_t SetGoalPosition(const vector<unsigned short>& goalPosition);
	size_t SetTorqueEnable(bool isEnabled);

	size_t GetPresentPosition(std::vector<unsigned short>& currentPosition);

private:
	DynamixelUART broadcastDynamixel;
	Uart* uart;
};

#endif
