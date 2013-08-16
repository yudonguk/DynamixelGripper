#include "DynamixelGroup.h"

#include <limits>

#include "DynamixelUARTDef.h"
#include "DummyDynamixelUART.h"

DynamixelGroup::DynamixelGroup( Uart* uart_ /*= NULL*/ )
	: uart(uart_), broadcastDynamixel(uart, ID_BROADCAST)
{
}

DynamixelGroup::~DynamixelGroup()
{
}

void DynamixelGroup::SetUart( Uart* uart_ )
{
	uart = uart_;
	broadcastDynamixel = DynamixelUART(uart, ID_BROADCAST);
}

size_t DynamixelGroup::SetGoalPosition( const vector<unsigned short>& goalPosition )
{
	if (size() * 3 > MAX_PAYLOAD_SIZE)
		return 0;

	unsigned char data[MAX_PAYLOAD_SIZE] = {0, };

	size_t payloadSize = 0;
	for (size_t i = 0;  i < size(); i++)
	{
		if(operator[](i)->id == DummyDynamixelUart::DUMMY_ID)
			continue;

		unsigned char* pPosition = (unsigned char*)(&goalPosition[i]);
		
		data[payloadSize++] = operator[](i)->id;
		data[payloadSize++] = pPosition[0];
		data[payloadSize++] = pPosition[1];
	}

	size_t result = 0;

	uart->Lock();
	if (broadcastDynamixel.WriteBytes(GOAL_POSITION_W, &data[0], payloadSize, 2, true))
		result = size();
	uart->Unlock();

	return result;
}

size_t DynamixelGroup::SetTorqueEnable( bool isEnabled )
{
	if (size() * 2 > MAX_PAYLOAD_SIZE)
		return 0;

	unsigned char data[MAX_PAYLOAD_SIZE] = {0, };

	size_t payloadSize = 0;
	for (size_t i = 0;  i < size(); i++)
	{
		if(operator[](i)->id == DummyDynamixelUart::DUMMY_ID)
			continue;

		data[payloadSize++] = operator[](i)->id;
		data[payloadSize++] = isEnabled ? 1 : 0;
	}

	size_t result = 0;

	uart->Lock();
	if (broadcastDynamixel.WriteBytes(TORQUE_EABLE, &data[0], payloadSize, 1, true))
		result = size();
	uart->Unlock();

	return result;
}

size_t DynamixelGroup::GetPresentPosition( std::vector<unsigned short>& currentPosition )
{
	currentPosition.resize(size());

	size_t result = 0;

	uart->Lock();
	for (size_t i = 0; i < size(); i++)
	{
		if(operator[](i)->id == DummyDynamixelUart::DUMMY_ID)
		{
			currentPosition[i] = 0;
			result |= 1 << i;
			continue;
		}

		unsigned short presentPositionRaw = 0;
		if (operator[](i)->GetPresentPosition(presentPositionRaw))
			result |= 1 << i;

		currentPosition[i] = presentPositionRaw;
	}
	uart->Unlock();

	return result;
}