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
	Clear();
}

void DynamixelGroup::SetUart( Uart* uart_ )
{
	uart = uart_;
	broadcastDynamixel = DynamixelUART(uart, ID_BROADCAST);
}

void DynamixelGroup::Clear()
{
	for (size_t i = 0; i < dynamixelVector.size(); i++)
	{
		delete dynamixelVector[i];
		dynamixelVector[i] = NULL;
	}
	dynamixelVector.clear();
}

size_t DynamixelGroup::SetGoalPosition( const vector<unsigned short>& goalPosition )
{
	vector<unsigned char> data;
	data.reserve(3 * goalPosition.size());

	for (size_t i = 0;  i < dynamixelVector.size(); i++)
	{
		if(dynamixelVector[i]->id == DummyDynamixelUart::DUMMY_ID)
			continue;

		unsigned char* pPosition = (unsigned char*)(&goalPosition[i]);

		data.push_back(dynamixelVector[i]->id);
		data.push_back(pPosition[0]);
		data.push_back(pPosition[1]);
	}

	uart->Lock();
	if (!broadcastDynamixel.WriteBytes(GOAL_POSITION_W, &data[0], data.size(), 2, true))
	{
		uart->Unlock();
		return 0;
	}
	uart->Unlock();

	return dynamixelVector.size();
}

size_t DynamixelGroup::SetTorqueEnable( bool isEnabled )
{
	vector<unsigned char> data;
	data.reserve(2 * dynamixelVector.size());

	for (size_t i = 0;  i < dynamixelVector.size(); i++)
	{
		if(dynamixelVector[i]->id == DummyDynamixelUart::DUMMY_ID)
			continue;

		data.push_back(dynamixelVector[i]->id);
		data.push_back(isEnabled ? 1 : 0);
	}

	uart->Lock();
	if (!broadcastDynamixel.WriteBytes(TORQUE_EABLE, &data[0], data.size(), 1, true))
	{
		uart->Unlock();
		return 0;
	}
	uart->Unlock();

	return dynamixelVector.size();
}

size_t DynamixelGroup::GetPresentPosition( std::vector<unsigned short>& currentPosition )
{
	currentPosition.resize(dynamixelVector.size());

	size_t result = 0;

	uart->Lock();
	for (size_t i = 0; i < dynamixelVector.size(); i++)
	{
		if(dynamixelVector[i]->id == DummyDynamixelUart::DUMMY_ID)
		{
			currentPosition[i] = 0;
			continue;
		}

		unsigned short presentPositionRaw = 0;
		if (dynamixelVector[i]->GetPresentPosition(presentPositionRaw))
			result |= 1 << i;

		currentPosition[i] = presentPositionRaw;
	}
	uart->Unlock();

	return result;
}
