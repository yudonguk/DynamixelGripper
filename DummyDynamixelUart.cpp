#include "DummyDynamixelUART.h"

DummyDynamixelUart::DummyDynamixelUart()
	: DynamixelUART(NULL, DUMMY_ID)
{
}

DummyDynamixelUart::~DummyDynamixelUart()
{
}

int DummyDynamixelUart::SendPacket( unsigned char *data, int dataSize )
{
	return 0;
}

int DummyDynamixelUart::ReceivePacket( unsigned char *data, int dataSize )
{
	memset(data, 0, dataSize);
	return 0;
}
