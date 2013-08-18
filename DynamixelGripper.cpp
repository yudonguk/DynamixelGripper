#include "DynamixelGripper.h"

#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/locks.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <device/OprosPrintMessage.h>
#include <OPRoSTypes.h>

#include "SerialCommunicator.h"
#include "DynamixelUARTDef.h"
#include "DummyDynamixelUART.h"

#define JOINT_COUNT				"JointCount"
#define COUNTERCLOCKWISE_MODE	"CounterclockwiseMode" 
#define DYNAMIXEL_ID			"DynamixelID"
#define COMPLIANCE_MARGINE		"ComplianceMargine"
#define COMPLIANCE_SLOPE		"ComplianceSlope"
#define POSITION_RESOLUTION		"PositionResolution"
#define POSITION_OFFSET			"PositionOffset"
#define MAXIMUM_POWER			"MaximumPower"
#define MAXIMUM_VELOCITY		"MaximumVelocity"
#define MINIMUM_POSITION_LIMIT	"MinimumPositionLimit"
#define MAXIMUM_POSITION_LIMIT	"MaximumPositionLimit"
#define MAXIMUM_LOAD			"MaximumLoad"

DynamixelGripper::DynamixelGripper()
	: uart(NULL), mIsGripped(false)
{}

DynamixelGripper::~DynamixelGripper()
{
	Finalize();
}

int DynamixelGripper::Initialize( Property parameter )
{
	if (Setting(parameter) == false)
	{
		PrintMessage("Error : DynamixelManipulator::Initialize()->Can't Initialize() << %s(%d)\n", __FILE__, __LINE__);
		_status = DEVICE_ERROR;
		return API_ERROR;
	}

	mJointPosition.resize(mDynamixelProperties.size());
	mDesiredJointPosition.resize(mDynamixelProperties.size());

	_status = DEVICE_READY;
	return API_SUCCESS;
}

int DynamixelGripper::Finalize()
{
	if (_status == DEVICE_ACTIVE)
		Disable();

	mDynamixelGroup.clear();

	if(uart != NULL)
	{
		uart->Finalize();
		delete uart;
		uart = NULL;
	}

	_status = DEVICE_CREATED;
	return API_SUCCESS;
}

int DynamixelGripper::Enable()
{
	if (_status == DEVICE_ACTIVE)
	{
		return API_SUCCESS;
	}
	else if(_status != DEVICE_READY)
	{
		PrintMessage("Error : DynamixelManipulator::Enable()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	bool error = false;
	for (size_t i = 0, end = mDynamixelProperties.size(); i < end; i++)
	{	
		DynamixelProperty& property = *mDynamixelProperties[i];
		//다이나믹셀 초기 설정
		if (EnableDynamixel(*property.pDynamixel, property) == false)
		{
			error = true;
		}
	}
	
	if (error)
	{
		_status = DEVICE_ERROR;
		return API_ERROR;
	}
	
	_status = DEVICE_ACTIVE;
	return API_SUCCESS;
}

int DynamixelGripper::Disable()
{
	if (_status == DEVICE_READY)
	{
		return API_SUCCESS;
	}
	else if(_status != DEVICE_ACTIVE)
	{
		PrintMessage("Error : DynamixelManipulator::Disable()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}
	
	bool error = false;
	for (size_t i = 0, end = mDynamixelProperties.size(); i < end; i++)
	{
		DynamixelProperty& property = *mDynamixelProperties[i];

		if(!property.pDynamixel->SetTorqueEnable(0))
		{
			PrintMessage("Error : DynamixelManipulator::Disable()->Can't disable Torque[%d]<< %s(%d)\r\n", i, __FILE__, __LINE__);
			error = true;
		}
		
		if (!property.pDynamixel->SetLED(0))
		{
			PrintMessage("Error : DynamixelManipulator::Disable()->Can't disable LED[%d]<< %s(%d)\r\n", i, __FILE__, __LINE__);
			error = true;
		}
	}

	if (error)
	{
		_status = DEVICE_ERROR;
		return API_ERROR;
	}
	_status =  DEVICE_READY;
	return API_SUCCESS;
}

bool DynamixelGripper::Setting( Property& parameter)
{
	if (uart != NULL)
	{
		uart->Finalize();
		delete uart;
		uart = NULL;
	}

	uart = new SerialCommunicator;

	if(uart->Initialize(parameter) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't Initialize UART<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}
	else if(uart->Enable() != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't Enable UART<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}

	mDynamixelProperties.clear();
	mDynamixelGroup.clear();
	mDynamixelGroup.SetUart(uart);

	if (parameter.FindName(JOINT_COUNT) == false)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't find Size<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}
	const size_t jointCount = boost::lexical_cast<size_t>(parameter.GetValue(JOINT_COUNT));
	
	PrintMessage("\r\nDynamixelGripper Property Setting\r\n");

	char buff[100] = {0, };
	for (size_t i = 0;  i < jointCount; i++)
	{
		boost::shared_ptr<DynamixelProperty> pDynamixelProperty = boost::make_shared<DynamixelProperty>();

		//CounterclockwiseMode
		sprintf(buff, "%s%d", COUNTERCLOCKWISE_MODE, i);
		if (parameter.FindName(buff)) 
			pDynamixelProperty->isCounterclockwiseMode 
			= boost::lexical_cast<bool>(parameter.GetValue(buff));
		PrintMessage("%s : %s \r\n", buff, pDynamixelProperty->isCounterclockwiseMode ? "true" : "false");

		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID, i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->id = boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->id);

		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE, i);
		if (parameter.FindName(buff)) 
			pDynamixelProperty->complianceMargine
			= boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->complianceMargine);

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE, i);
		if (parameter.FindName(buff)) 
			pDynamixelProperty->compliacneSlope
			= boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->compliacneSlope);

		//PositionResolution
		sprintf(buff, "%s%d", POSITION_RESOLUTION, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->positionResolution
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->positionResolution);

		//PositionOffset
		sprintf(buff, "%s%d", POSITION_OFFSET, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->positionOffset
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->positionOffset);

		//MaximumPower
		sprintf(buff, "%s%d", MAXIMUM_POWER, i);
		if (parameter.FindName(buff))
			pDynamixelProperty->maximumPower
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->maximumPower);

		//MaximumVelocity
		sprintf(buff, "%s%d", MAXIMUM_VELOCITY, i);
		if (parameter.FindName(buff)) 
			pDynamixelProperty->maximuVelocity
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->maximuVelocity);

		//MinimumPositionLimit
		sprintf(buff, "%s%d", MINIMUM_POSITION_LIMIT, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		} 
		pDynamixelProperty->minimumPositionLimit
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->minimumPositionLimit);

		//MaximumPositionLimit
		sprintf(buff, "%s%d", MAXIMUM_POSITION_LIMIT, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->maximumPositionLimit
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->maximumPositionLimit);

		PrintMessage("\r\n");
		
		if(pDynamixelProperty->id == DummyDynamixelUart::DUMMY_ID)
			pDynamixelProperty->pDynamixel = boost::make_shared<DummyDynamixelUart>();
		else
			pDynamixelProperty->pDynamixel = boost::make_shared<DynamixelUART>(uart, pDynamixelProperty->id);

		mDynamixelProperties.push_back(pDynamixelProperty);
		mDynamixelGroup.push_back(pDynamixelProperty->pDynamixel);
	}

	{
		boost::shared_ptr<GripperDynamixelProperty> pGripperProperty = boost::make_shared<GripperDynamixelProperty>();

		bool isEnoughGripperProperty = true;

		//CounterclockwiseMode
		sprintf(buff, "Gripper%s", COUNTERCLOCKWISE_MODE);
		if (parameter.FindName(buff)) 
			pGripperProperty->isCounterclockwiseMode 
			= boost::lexical_cast<bool>(parameter.GetValue(buff));
		PrintMessage("%s : %s \r\n", buff, pGripperProperty->isCounterclockwiseMode ? "true" : "false");

		//DynamixelID
		sprintf(buff, "Gripper%s", DYNAMIXEL_ID);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty = false;
		}
		else
		{
			pGripperProperty->id = boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
			PrintMessage("%s : %d \r\n", buff, pGripperProperty->id);	
		}
		
		//ComplianceMargine
		sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE);
		if (parameter.FindName(buff)) 
			pGripperProperty->complianceMargine
			= boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pGripperProperty->complianceMargine);	
		
		//ComplianceSlope
		sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE);
		if (parameter.FindName(buff)) 
			pGripperProperty->compliacneSlope
			= boost::lexical_cast<unsigned char>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pGripperProperty->compliacneSlope);	

		//PositionResolution
		sprintf(buff, "Gripper%s", POSITION_RESOLUTION);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pGripperProperty->positionResolution
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pGripperProperty->positionResolution);	
		}

		//PositionOffset
		sprintf(buff, "Gripper%s", POSITION_OFFSET);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pGripperProperty->positionOffset
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pGripperProperty->positionOffset);	
		}

		//MaximumPower
		sprintf(buff, "Gripper%s", MAXIMUM_POWER);
		if (parameter.FindName(buff))
			pGripperProperty->maximumPower
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumPower);	

		//MaximumVelocity
		sprintf(buff, "Gripper%s", MAXIMUM_VELOCITY);
		if (parameter.FindName(buff)) 
			pGripperProperty->maximuVelocity
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximuVelocity);	

		//MinimumPositionLimit
		sprintf(buff, "Gripper%s", MINIMUM_POSITION_LIMIT);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pGripperProperty->minimumPositionLimit
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pGripperProperty->minimumPositionLimit);	
		}

		//MaximumPositionLimit
		sprintf(buff, "Gripper%s", MAXIMUM_POSITION_LIMIT);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pGripperProperty->maximumPositionLimit
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumPositionLimit);	
		}

		//MaximumLoad
		sprintf(buff, "GripperMaximumLoad");
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pGripperProperty->maximumLoad
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumLoad);	
		}

		if (isEnoughGripperProperty)
		{	
			pGripperProperty->pDynamixel = boost::make_shared<DynamixelUART>(uart, pGripperProperty->id);
		}
		else
		{
			pGripperProperty->id = DummyDynamixelUart::DUMMY_ID;
			pGripperProperty->pDynamixel =  boost::make_shared<DummyDynamixelUart>();

			PrintMessage("DynamixelGripper will operate Manipulator. \r\n");
			PrintMessage("Because property of related gripper is not enough. \r\n");			
		}

		PrintMessage("\r\n");	
				
		mDynamixelProperties.push_back(pGripperProperty);
		mDynamixelGroup.push_back(pGripperProperty->pDynamixel);
	}

	return true;
}

int DynamixelGripper::SetParameter( Property parameter )
{
	if (_status == DEVICE_ACTIVE)
	{
		Disable();
	}
	else if(_status != DEVICE_READY)
	{
		PrintMessage("Error : DynamixelManipulator::SetParameter()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (Setting(parameter) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SetParameter()->Can't set parameter.<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (Enable() != API_SUCCESS)
	{
		return API_ERROR;
	}

	return API_SUCCESS;
}

bool DynamixelGripper::EnableDynamixel( DynamixelUART& dynamixel, const DynamixelProperty& property )
{
	if(dynamixel.SetStatusReturnLevel(1) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set _statusRetrunLevel of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if (dynamixel.SetCCWComplianceMargin(property.complianceMargine) == false
		|| dynamixel.SetCWComplianceMargin(property.complianceMargine) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set ComplianceMargin of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetCCWComplianceSlope(property.compliacneSlope) == false
		|| dynamixel.SetCWComplianceSlope(property.compliacneSlope) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set ComplianceSlope of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetMaximumTorque(ConvertPowerUnitToDynamixel(property.maximumPower)) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set MaximumPower of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetMovingSpeed(ConvertVelocityUnitToDynamixel(property.maximuVelocity * 60 / 360)) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set MaximumVelocity of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetCCWAngleLimit(ConvertPositionUnitToDynamixel(property.maximumPositionLimit, property.positionOffset, property.positionResolution)) == false
		|| dynamixel.SetCWAngleLimit(ConvertPositionUnitToDynamixel(property.minimumPositionLimit, property.positionOffset, property.positionResolution)) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't set PositionLimit of ID(%d)<< %s(%d)\r\n", property.id,__FILE__, __LINE__);
		return false;
	}
	else if (dynamixel.SetLED(1) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't enable LED of ID(%d)<< %s(%d)\r\n", property.id, __FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetAlarmShutdown(OVERLOAD_ERROR | OVERHEATING_ERROR | INPUTVOLTAGE_ERROR) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't enable AlarmShutdown of ID(%d)<< %s(%d)\r\n", property.id, __FILE__, __LINE__);
		return false;
	}
	else if(dynamixel.SetTorqueEnable(1) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SettingDynamixel()->Can't enable Torque of ID(%d)<< %s(%d)\r\n", property.id, __FILE__, __LINE__);
		return false;
	}

	return true;
}

int DynamixelGripper::GetParameter( Property& parameter )
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::GetParameter()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (uart->GetParameter(parameter) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::GetParameter()->Can't get parameter to UART.<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	parameter.SetValue(JOINT_COUNT, boost::lexical_cast<std::string>(mDynamixelProperties.size() - 1));
	
	char buff[100] = {0, };

	for (size_t i = 0, end = mDynamixelProperties.size() - 1;  i < end; i++)
	{
		DynamixelProperty& property = *mDynamixelProperties[i];

		//CounterclockwiseMode
		sprintf(buff, "%s%d", COUNTERCLOCKWISE_MODE, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.isCounterclockwiseMode));
		
		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.id));
		
		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.complianceMargine));

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.compliacneSlope));

		//PositionResolution
		sprintf(buff, "%s%d", POSITION_RESOLUTION, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.positionResolution));

		//PositionOffset
		sprintf(buff, "%s%d", POSITION_OFFSET, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.positionOffset));

		//MaximumPower
		sprintf(buff, "%s%d", MAXIMUM_POWER, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximumPower));

		//MaximumVelocity
		sprintf(buff, "%s%d", MAXIMUM_VELOCITY, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximuVelocity));

		//MinimumPositionLimit
		sprintf(buff, "%s%d", MINIMUM_POSITION_LIMIT, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.minimumPositionLimit));

		//MaximumPositionLimit
		sprintf(buff, "%s%d", MAXIMUM_POSITION_LIMIT, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximumPositionLimit));
	}

	{
		GripperDynamixelProperty& property 
			= static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
		
		if (property.id != DummyDynamixelUart::DUMMY_ID)
		{
			//CounterclockwiseMode
			sprintf(buff, "Gripper%s", COUNTERCLOCKWISE_MODE);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.isCounterclockwiseMode));

			//DynamixelID
			sprintf(buff, "Gripper%s", DYNAMIXEL_ID);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.id));

			//ComplianceMargine
			sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.complianceMargine));

			//ComplianceSlope
			sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.compliacneSlope));

			//PositionResolution
			sprintf(buff, "Gripper%s", POSITION_RESOLUTION);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.positionResolution));

			//PositionOffset
			sprintf(buff, "Gripper%s", POSITION_OFFSET);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.positionOffset));

			//MaximumPower
			sprintf(buff, "Gripper%s", MAXIMUM_POWER);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximumPower));

			//MaximumVelocity
			sprintf(buff, "Gripper%s", MAXIMUM_VELOCITY);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximuVelocity));

			//MinimumPositionLimit
			sprintf(buff, "Gripper%s", MINIMUM_POSITION_LIMIT);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.minimumPositionLimit));

			//MaximumPositionLimit
			sprintf(buff, "Gripper%s", MAXIMUM_POSITION_LIMIT);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximumPositionLimit));	

			//MaximumLoad
			sprintf(buff, "Gripper%s", MAXIMUM_LOAD);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(property.maximumLoad));
		}
	}

	return API_SUCCESS;
}

int DynamixelGripper::OnExecute()
{
	if(_status != DEVICE_ACTIVE)
	{
		PrintMessage("Error : DynamixelManipulator::OnExecute()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	UpdateJointState();
	ControlJoint();

	return API_SUCCESS;
}

int DynamixelGripper::RunHoming()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::StartHoming()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	//모든 위치를 0으로 
	std::vector<double> position(mDynamixelProperties.size() - 1);
	std::vector<unsigned long> time(position.size());

	if (SetPosition(position,time) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::StartHoming()->Can't StartHoming Dynamixel<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}
	
	return API_SUCCESS;
}

int DynamixelGripper::Stop()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	std::vector<double> position;

	if (GetPosition(position) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Can't Stop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}
	
	std::vector<unsigned long> time(position.size());

	if (SetPosition(position, time) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Can't Stop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}
	return API_SUCCESS;
}

int DynamixelGripper::EmergencyStop()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::EmergencyStop()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	uart->Lock();
	if (mDynamixelGroup.SetTorqueEnable(false) == false)
	{
		PrintMessage("Error : DynamixelManipulator::EmergencyStop()->Can't EmergencyStop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		uart->Unlock();
		return API_ERROR;
	}
	uart->Unlock();
	return API_SUCCESS;
}

int DynamixelGripper::SetPosition( vector<double> position, vector<unsigned long> time )
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (position.size() != mDynamixelProperties.size() - 1)
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->position size must be equal Dynamixel count<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (time.size() != position.size())
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->time size must be equal Dynamixel count<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	boost::unique_lock<boost::shared_mutex> lock(mJointPositionMutex);

	std::copy(position.begin(), position.end(), mDesiredJointPosition.begin());

	return API_SUCCESS;
}

int DynamixelGripper::GetPosition( vector<double> &position )
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::GetPosition()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	boost::shared_lock<boost::shared_mutex> lock(mJointPositionMutex);

	// mJointPosition의 마지막 원소는 그리퍼 조인트의 위치 이므로,
	position.resize(mJointPosition.size() - 1);
	std::copy(mJointPosition.begin(), mJointPosition.end() - 1, position.begin());

	return API_SUCCESS;
}

int DynamixelGripper::StartGripping()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::IsGripped()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	
	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	uart->Lock();
	property.pDynamixel->SetGoalPosition(ConvertPositionUnitToDynamixel(
		(property.isCounterclockwiseMode ? 1.0 : -1.0) * property.maximumPositionLimit
		, property.positionOffset, property.positionResolution));
	uart->Unlock();

	mIsGripped = true;

	return API_SUCCESS;
}

int DynamixelGripper::StopGripping()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::IsGripped()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	//gripperMessageQueue.Push(STOP_GRIPPING);
	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	uart->Lock();
	property.pDynamixel->SetGoalPosition(ConvertPositionUnitToDynamixel(
		(property.isCounterclockwiseMode ? 1.0 : -1.0) * property.minimumPositionLimit
		, property.positionOffset, property.positionResolution));
	uart->Unlock();

	mIsGripped = false;
	return API_SUCCESS;
}

int DynamixelGripper::IsGripped(bool &isGripped)
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::IsGripped()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	GripperDynamixelProperty& property
		= static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	isGripped = mIsGripped;
	return API_SUCCESS;

	boost::shared_lock<boost::shared_mutex> lock(mJointPositionMutex);
	std::cout << mGripperJointLoad << std::endl;

	if (mGripperJointLoad > property.maximumLoad * 0.9)
	{
		return 1;
	}

	return API_SUCCESS;
}

unsigned short DynamixelGripper::ConvertPowerUnitToDynamixel( const double& percent)
{
	int dynamixelValue = int(percent / 0.1);
	return std::min(std::max(dynamixelValue, 0), 1023);
}

unsigned short DynamixelGripper::ConvertPositionUnitToDynamixel( const double& degree, const double& offset, const double& resolution)
{
	int dynamixelValue = int((degree + offset) / resolution);
	return std::min(std::max(dynamixelValue, 0), 4095);
}

unsigned short DynamixelGripper::ConvertVelocityUnitToDynamixel( const double& rpm )
{
	int dynamixelValue = int(rpm / 0.111);
	return std::min(std::max(dynamixelValue, 0), 1023);
}

double DynamixelGripper::ConvertPowerUnitToPercent( unsigned short dynamixelValue )
{
	double percent = std::min(int(dynamixelValue), 1023) * 0.1;
	return percent;
}

double DynamixelGripper::ConvertPositionUnitToDegree( unsigned short dynamixelValue, const double& offset, const double& resolution )
{
	double degree = std::min(std::max(int(dynamixelValue), 0), 4095) * resolution - offset;
	return degree;
}

double DynamixelGripper::ConvertVelocityUnitToRPM( unsigned short dynamixelValue )
{
	return std::min(std::max(int(dynamixelValue), 0), 1023) * 0.111;
}

double DynamixelGripper::ConvertLoadUnitToPercent( unsigned short dynamixelValue )
{
	dynamixelValue = std::min(std::max(int(dynamixelValue & 0x3FF), 0), 2047);
	double percent = (dynamixelValue & 0x3FF) * 0.1 * (dynamixelValue & 0x400 ? 1.0 : -1.0);
	return percent;
}


void DynamixelGripper::UpdateJointState()
{
	std::vector<unsigned short> rawJointPosition;
	std::vector<double> jointPosition(mJointPosition.size());
	unsigned short rawGripperJointLoad = 0;

	DynamixelProperty& gripperProperty = **mDynamixelProperties.rbegin();

	uart->Lock();
	size_t positionResult = mDynamixelGroup.GetPresentPosition(rawJointPosition);
	bool resultOfGettingGripperLoad = gripperProperty.pDynamixel->GetPresentLoad(rawGripperJointLoad);
	uart->Unlock();

	for (size_t i = 0, end = mDynamixelProperties.size(); i < end; i++)
	{
		if (!(positionResult & (1 << i)))
		{
			mJointPositionMutex.lock_shared();

			// 조인트의 위치를 가져오지 못할 경우, 이전 조인트 위치를 적용한다.
			jointPosition[i] = mJointPosition[i];

			mJointPositionMutex.unlock_shared();

			PrintMessage("Error : DynamixelManipulator::GetPosition()->Can't GetPosition Dynamixel[%d]<< %s(%d)\r\n", i, __FILE__, __LINE__);
			continue;
		}

		DynamixelProperty& property = *mDynamixelProperties[i];

		// 단위계 변환
		jointPosition[i] = (property.isCounterclockwiseMode ? 1.0 : -1.0)
			* ConvertPositionUnitToDegree(rawJointPosition[i]
		, property.positionOffset, property.positionResolution);
	}

	boost::unique_lock<boost::shared_mutex> lock(mJointPositionMutex);
	mJointPosition = std::move(jointPosition);
	// 그리퍼 조인트의 하중을 얻어왔을 경우에만 갱신
	if(resultOfGettingGripperLoad)
		mGripperJointLoad = (gripperProperty.isCounterclockwiseMode ? 1.0 : -1.0)
		* ConvertLoadUnitToPercent(rawGripperJointLoad);
}

void DynamixelGripper::ControlJoint()
{
	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mJointPositionMutex);
	GripperDynamixelProperty& gripperProperty = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	
	// 그리퍼 제어
	if (mGripperCommand == START_GRIPPING)
	{
		// 따라가야할 값과 조작해야할 값의 방향이 반대여서 아래와 같이 처리함
		const double loadError = mGripperJointLoad - gripperProperty.maximumLoad;
		const boost::chrono::high_resolution_clock::time_point now 
			= boost::chrono::high_resolution_clock::now();
		const double periode 
			= boost::chrono::duration_cast<boost::chrono::duration<double>>(now - mGripperLoadPIControl.time).count();

		double manipulatedValue = mGripperLoadPIControl.manipulatedValue
			+ mGripperLoadPIControl.kp * (loadError - mGripperLoadPIControl.error)
			+ mGripperLoadPIControl.ki * periode * loadError;
		
		manipulatedValue = std::min(std::max(manipulatedValue, gripperProperty.minimumPositionLimit)
			, gripperProperty.maximumPositionLimit);

		// 다음 계산을 위해 저장
		mGripperLoadPIControl.time = now;
		mGripperLoadPIControl.error = loadError;
		mGripperLoadPIControl.manipulatedValue = manipulatedValue;
		
		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		*mDesiredJointPosition.rbegin() = manipulatedValue;
	}
	else
	{
		mGripperLoadPIControl.time = boost::chrono::high_resolution_clock::now();
		mGripperLoadPIControl.error = 0.0;
		mGripperLoadPIControl.manipulatedValue = gripperProperty.minimumPositionLimit;

		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		*mDesiredJointPosition.rbegin() = gripperProperty.minimumPositionLimit;
	}	

	//단위 변환
	vector<unsigned short> rawJointPosition(mDesiredJointPosition.size());
	for (size_t i = 0, end = rawJointPosition.size();  i < end; i++)
	{
		DynamixelProperty& property = *mDynamixelProperties[i];

		rawJointPosition[i] = ConvertPositionUnitToDynamixel(
			(property.isCounterclockwiseMode ? 1.0 : -1.0) * mDesiredJointPosition[i]
		, mDynamixelProperties[i]->positionOffset, mDynamixelProperties[i]->positionResolution);
	}

	upgradeLock.unlock();

	// 조인트 위치 설정
	uart->Lock();
	if (!mDynamixelGroup.SetGoalPosition(rawJointPosition))
	{
		PrintMessage("Error : DynamixelManipulator::ControlJoint()->Can't SetPosition Dynamixel<< %s(%d)\r\n", __FILE__, __LINE__);
	}
	uart->Unlock();
}

#ifdef WIN32
extern "C"
{
	__declspec(dllexport) OprosApi* GetAPI();
}

OprosApi* GetAPI()
{
	return new DynamixelGripper();
}

#else
extern "C"
{
	OprosApi* GetAPI();
}

OprosApi* GetAPI()
{
	return new SharedUART;
}

#endif

#undef JOINT_COUNT
#undef COUNTERCLOCKWISE_MODE
#undef DYNAMIXEL_ID
#undef COMPLIANCE_MARGINE
#undef COMPLIANCE_SLOPE
#undef POSITION_RESOLUTION
#undef POSITION_OFFSET
#undef MAXIMUM_POWER
#undef MAXIMUM_VELOCITY
#undef MINIMUM_POSITION_LIMIT
#undef MAXIMUM_POSITION_LIMIT
#undef MAXIMUM_LOAD