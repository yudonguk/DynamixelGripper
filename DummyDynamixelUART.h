#ifndef __DUMMY_DYNAMIXEL_UART_H__
#define __DUMMY_DYNAMIXEL_UART_H__

#include "DynamixelUART.h"

class DummyDynamixelUart : public DynamixelUART
{
public:
	const static unsigned char DUMMY_ID = 0xFF;

public:
	DummyDynamixelUart();	
	virtual ~DummyDynamixelUart();

public:
	virtual int SendPacket (unsigned char *data, int dataSize);
	virtual int ReceivePacket (unsigned char *data, int dataSize);
};

#endif // !__DUMMY_DYNAMIXEL_UART_H__
