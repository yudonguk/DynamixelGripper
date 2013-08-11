#include "DynamixelGroup.h"

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

void DynamixelGroup::Add( DynamixelUART* pDynamixel )
{
	dynamixelVector.push_back(pDynamixel);
}

size_t DynamixelGroup::CountDynamixel()
{
	return dynamixelVector.size();
}

DynamixelUART& DynamixelGroup::operator[]( size_t index )
{
	return *dynamixelVector[index];
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

bool DynamixelGroup::SetGoalPosition( const vector<unsigned short>& goalPosition )
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

	return broadcastDynamixel.WriteBytes(GOAL_POSITION_W, &data[0], data.size(), 2, true);
}

bool DynamixelGroup::SetTorqueEnable( bool isEnabled )
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

	return broadcastDynamixel.WriteBytes(GOAL_POSITION_W, &data[0], data.size(), 1, true);
}
