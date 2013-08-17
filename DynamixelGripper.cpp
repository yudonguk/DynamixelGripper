#include "DynamixelGripper.h"

#include <algorithm>

#include <device/ServoActuator.h>
#include <device/OprosPrintMessage.h>

#include "SerialCommunicator.h"
#include "DynamixelUARTDef.h"
#include "DummyDynamixelUART.h"

DynamixelGripper::DynamixelGripper()
	: uart(NULL), gripperControlThread(NULL), mIsGripped(false)
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

	mJointPosition.resize(mDynamixelGroup.size());

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

	gripperControlThread = new boost::thread(boost::bind(&DynamixelGripper::GripperControlThreadHandler, this));

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

	if (gripperControlThread != NULL)
	{
		gripperControlThread->interrupt();
		gripperControlThread->timed_join(boost::posix_time::millisec(100));
		delete gripperControlThread;
		gripperControlThread = NULL;
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

	if (parameter.FindName("Size") == false)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't find Size<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}
	const size_t jointCount = atoi(parameter.GetValue("Size").c_str());

	const string DYNAMIXEL_ID = "DynamixelID";
	const string COMPLIANCE_MARGINE = "ComplianceMargine";
	const string COMPLIANCE_SLOPE = "ComplianceSlope";
	const string POSITION_RESOLUTION = "PositionResolution";
	const string POSITION_OFFSET = "PositionOffset";
	const string MAXIMUM_POWER = "MaximumPower";
	const string MAXIMUM_VELOCITY = "MaximumVelocity";
	const string MINIMUM_POSITION_LIMIT = "MinimumPositionLimit";
	const string MAXIMUM_POSITION_LIMIT = "MaximumPositionLimit";

	PrintMessage("\r\nDynamixelGripper Property Setting\r\n");

	char buff[100] = {0, };
	for (unsigned int i = 0;  i < jointCount; i++)
	{
		boost::shared_ptr<DynamixelProperty> pDynamixelProperty = boost::make_shared<DynamixelProperty>();

		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->id = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->id);

		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->complianceMargine = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->complianceMargine);

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->compliacneSlope = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pDynamixelProperty->compliacneSlope);

		//PositionResolution
		sprintf(buff, "%s%d", POSITION_RESOLUTION.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->positionResolution = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->positionResolution);

		//PositionOffset
		sprintf(buff, "%s%d", POSITION_OFFSET.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->positionOffset = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->positionOffset);

		//MaximumPower
		sprintf(buff, "%s%d", MAXIMUM_POWER.c_str(), i);
		if (parameter.FindName(buff) == false)
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->maximumPower = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->maximumPower);

		//MaximumVelocity
		sprintf(buff, "%s%d", MAXIMUM_VELOCITY.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->maximuVelocity = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->maximuVelocity);

		//MinimumPositionLimit
		sprintf(buff, "%s%d", MINIMUM_POSITION_LIMIT.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->minimumPositionLimit = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pDynamixelProperty->minimumPositionLimit);

		//MaximumPositionLimit
		sprintf(buff, "%s%d", MAXIMUM_POSITION_LIMIT.c_str(), i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pDynamixelProperty->maximumPositionLimit = atof(parameter.GetValue(buff).c_str());
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

		//DynamixelID
		sprintf(buff, "Gripper%s", DYNAMIXEL_ID.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->id = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pGripperProperty->id);	

		//ComplianceMargine
		sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->complianceMargine = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pGripperProperty->complianceMargine);	

		//ComplianceSlope
		sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->compliacneSlope = atoi(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %d \r\n", buff, pGripperProperty->compliacneSlope);	

		//PositionResolution
		sprintf(buff, "Gripper%s", POSITION_RESOLUTION.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->positionResolution = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->positionResolution);	

		//PositionOffset
		sprintf(buff, "Gripper%s", POSITION_OFFSET.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->positionOffset = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->positionOffset);	

		//MaximumPower
		sprintf(buff, "Gripper%s", MAXIMUM_POWER.c_str());
		if (parameter.FindName(buff) == false)
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->maximumPower = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumPower);	

		//MaximumVelocity
		sprintf(buff, "Gripper%s", MAXIMUM_VELOCITY.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->maximuVelocity = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximuVelocity);	

		//MinimumPositionLimit
		sprintf(buff, "Gripper%s", MINIMUM_POSITION_LIMIT.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->minimumPositionLimit = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->minimumPositionLimit);	

		//MaximumPositionLimit
		sprintf(buff, "Gripper%s", MAXIMUM_POSITION_LIMIT.c_str());
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->maximumPositionLimit = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumPositionLimit);	

		//MaximumLoad
		sprintf(buff, "GripperMaximumLoad");
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pGripperProperty->maximumLoad = atof(parameter.GetValue(buff).c_str());
		PrintMessage("%s : %lf \r\n", buff, pGripperProperty->maximumLoad);	

		PrintMessage("\r\n");	

		pGripperProperty->pDynamixel = boost::make_shared<DynamixelUART>(uart, pGripperProperty->id);
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

	const string DYNAMIXEL_ID = "DynamixelID";
	const string COMPLIANCE_MARGINE = "ComplianceMargine";
	const string COMPLIANCE_SLOPE = "ComplianceSlope";
	const string POSITION_RESOLUTION = "PositionResolution";
	const string POSITION_OFFSET = "PositionOffset";
	const string MAXIMUM_POWER = "MaximumPower";
	const string MAXIMUM_VELOCITY = "MaximumVelocity";
	const string MINIMUM_POSITION_LIMIT = "MinimumPositionLimit";
	const string MAXIMUM_POSITION_LIMIT = "MaximumPositionLimit";
	
	char buff[100] = {0, };
	stringstream stringStream;

	for (size_t i = 0, end = mDynamixelProperties.size() - 1;  i < end; i++)
	{
		DynamixelProperty& dynamixelProperty = *mDynamixelProperties[i];

		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID, i);
		stringStream.str("");
		stringStream << dynamixelProperty.id;
		parameter.SetValue(buff, stringStream.str());

		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.complianceMargine;
		parameter.SetValue(buff, stringStream.str());

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.compliacneSlope;
		parameter.SetValue(buff, stringStream.str());

		//PositionResolution
		sprintf(buff, "%s%d", POSITION_RESOLUTION.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.positionResolution;
		parameter.SetValue(buff, stringStream.str());

		//PositionOffset
		sprintf(buff, "%s%d", POSITION_OFFSET.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.positionOffset;
		parameter.SetValue(buff, stringStream.str());

		//MaximumPower
		sprintf(buff, "%s%d", MAXIMUM_POWER.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.maximumPower;
		parameter.SetValue(buff, stringStream.str());

		//MaximumVelocity
		sprintf(buff, "%s%d", MAXIMUM_VELOCITY.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.maximuVelocity;
		parameter.SetValue(buff, stringStream.str());

		//MinimumPositionLimit
		sprintf(buff, "%s%d", MINIMUM_POSITION_LIMIT.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.minimumPositionLimit;
		parameter.SetValue(buff, stringStream.str());

		//MaximumPositionLimit
		sprintf(buff, "%s%d", MAXIMUM_POSITION_LIMIT.c_str(), i);
		stringStream.str("");
		stringStream << dynamixelProperty.maximumPositionLimit;
		parameter.SetValue(buff, stringStream.str());
	}

	{
		GripperDynamixelProperty& gripperProperty = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());

		//DynamixelID
		sprintf(buff, "Gripper%s", DYNAMIXEL_ID.c_str());
		stringStream.str("");
		stringStream << gripperProperty.id;
		parameter.SetValue(buff, stringStream.str());

		//ComplianceMargine
		sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE.c_str());
		stringStream.str("");
		stringStream << gripperProperty.complianceMargine;
		parameter.SetValue(buff, stringStream.str());

		//ComplianceSlope
		sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE.c_str());
		stringStream.str("");
		stringStream << gripperProperty.compliacneSlope;
		parameter.SetValue(buff, stringStream.str());

		//PositionResolution
		sprintf(buff, "Gripper%s", POSITION_RESOLUTION.c_str());
		stringStream.str("");
		stringStream << gripperProperty.positionResolution;
		parameter.SetValue(buff, stringStream.str());

		//PositionOffset
		sprintf(buff, "Gripper%s", POSITION_OFFSET.c_str());
		stringStream.str("");
		stringStream << gripperProperty.positionOffset;
		parameter.SetValue(buff, stringStream.str());

		//MaximumPower
		sprintf(buff, "Gripper%s", MAXIMUM_POWER.c_str());
		stringStream.str("");
		stringStream << gripperProperty.maximumPower;
		parameter.SetValue(buff, stringStream.str());

		//MaximumVelocity
		sprintf(buff, "Gripper%s", MAXIMUM_VELOCITY.c_str());
		stringStream.str("");
		stringStream << gripperProperty.maximuVelocity;
		parameter.SetValue(buff, stringStream.str());

		//MinimumPositionLimit
		sprintf(buff, "Gripper%s", MINIMUM_POSITION_LIMIT.c_str());
		stringStream.str("");
		stringStream << gripperProperty.minimumPositionLimit;
		parameter.SetValue(buff, stringStream.str());

		//MaximumPositionLimit
		sprintf(buff, "Gripper%s", MAXIMUM_POSITION_LIMIT.c_str());
		stringStream.str("");
		stringStream << gripperProperty.maximumPositionLimit;
		parameter.SetValue(buff, stringStream.str());	

		//MaximumLoad
		sprintf(buff, "GripperMaximumLoad");
		stringStream.str("");
		stringStream << gripperProperty.maximumLoad;
		parameter.SetValue(buff, stringStream.str());
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

	std::vector<unsigned short> rawJointPosition(mDynamixelProperties.size());
	std::vector<double> jointPosition(mDynamixelProperties.size());
	unsigned short rawGripperJointLoad = 0;

	uart->Lock();
	size_t positionResult = mDynamixelGroup.GetPresentPosition(rawJointPosition);
	bool resultOfGettingGripperLoad = (*mDynamixelGroup.rbegin())->GetPresentLoad(rawGripperJointLoad);
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
		jointPosition[i] = ConvertPositionUnitToDegree(rawJointPosition[i]
		, property.positionOffset, property.positionResolution);
	}

	boost::unique_lock<boost::shared_mutex> lock(mJointPositionMutex);
	mJointPosition = std::move(jointPosition);
	// 그리퍼 조인트의 하중을 얻어왔을 경우에만 갱신
	if(resultOfGettingGripperLoad)
		mGripperJointLoad = -ConvertLoadUnitToPercent(rawGripperJointLoad);

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
	vector<double> position(mDynamixelGroup.size() - 1);
	vector<unsigned long> time(position.size());

	uart->Lock();
	if (SetPosition(position, time) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::StartHoming()->Can't StartHoming Dynamixel<< %s(%d)\r\n", __FILE__, __LINE__);
		uart->Unlock();
		return API_ERROR;
	}
	uart->Unlock();
	return API_SUCCESS;
}

int DynamixelGripper::Stop()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	vector<double> position;
	vector<unsigned long> time;

	uart->Lock();
	if (GetPosition(position) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Can't Stop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		uart->Unlock();
		return API_ERROR;
	}

	time.resize(position.size());

	if (SetPosition(position, time) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Stop()->Can't Stop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		uart->Unlock();
		return API_ERROR;
	}
	uart->Unlock();
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

	if (position.size() != mDynamixelGroup.size())
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->position size must be equal Dynamixel count<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	if (time.size() != position.size())
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->time size must be equal Dynamixel count<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	//단위 변환
	vector<unsigned short> dynamixelPositon(position.size());
	for (size_t i = 0, end = dynamixelPositon.size();  i < end; i++)
	{
		DynamixelProperty& property = *mDynamixelProperties[i];
		dynamixelPositon[i] = ConvertPositionUnitToDynamixel(position[i]
		, property.positionOffset, property.positionResolution);
	}

	uart->Lock();
	if (mDynamixelGroup.SetGoalPosition(dynamixelPositon) == false)
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->Can't SetPosition Dynamixel<< %s(%d)\r\n", __FILE__, __LINE__);
		uart->Unlock();
		return API_ERROR;
	}
	uart->Unlock();

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
	// GetPosition() 에서는 그리퍼 조인트의 위치를 반환하지 않는다.
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

	DynamixelUART& gripper = **mDynamixelGroup.rbegin();

	uart->Lock();
	GripperDynamixelProperty& gripperProperty = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	gripper.SetGoalPosition(ConvertPositionUnitToDynamixel(gripperProperty.maximumPositionLimit, gripperProperty.positionOffset, gripperProperty.positionResolution));
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
	DynamixelUART& gripper = **mDynamixelGroup.rbegin();

	uart->Lock();
	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	gripper.SetGoalPosition(ConvertPositionUnitToDynamixel(property.minimumPositionLimit, property.positionOffset, property.positionResolution));
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
	isGripped = mIsGripped;
	return API_SUCCESS;

	boost::shared_lock<boost::shared_mutex> lock(mJointPositionMutex);
	std::cout << mGripperJointLoad << std::endl;

	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());

	if (mGripperJointLoad > property.maximumLoad * 0.9)
	{
		return 1;
	}

	return API_SUCCESS;
}

void DynamixelGripper::GripperControlThreadHandler()
{
	return;

	try
	{
		GripperCommand command;
		for (;;)
		{	
			command = gripperMessageQueue.Pop();

			switch(command)
			{
			case START_GRIPPING:
				{
					double presentPosition = 1;
					double presentLoad = 1;

					struct ScopedLock
					{
						ScopedLock(Uart* pUart)
							: mpUart(pUart)
						{mpUart->Lock();}

						~ScopedLock(){mpUart->Unlock();}

						Uart* mpUart;
					};

					try
					{
						GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
						for(;; boost::this_thread::sleep(boost::posix_time::millisec(100)))
						{
							boost::shared_lock<boost::shared_mutex> lock(mJointPositionMutex);

							mJointPositionMutex.lock_shared();

							double presentLoad = mGripperJointLoad;
							double presentPosition = *mJointPosition.rbegin();

							mJointPositionMutex.unlock_shared();

							if (presentLoad > property.maximumLoad * 0.9)
							{
								break;
							}

							double loadDifference = property.maximumLoad - presentLoad;
							double positionDifference = (property.maximumPositionLimit - property.minimumPositionLimit) / 7 * (loadDifference / property.maximumLoad);

							DynamixelUART& gripper = **mDynamixelGroup.rbegin();
							gripper.SetGoalPosition(ConvertPositionUnitToDynamixel(presentPosition + positionDifference, property.positionOffset, property.positionResolution));
						}
					}
					catch(...)
					{
					}
				}
				break;
			case  STOP_GRIPPING:
				{
					uart->Lock();
					GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
					property.pDynamixel->SetGoalPosition(ConvertPositionUnitToDynamixel(property.minimumPositionLimit, property.positionOffset, property.positionResolution));
					uart->Unlock();
				}
				break;
			}
		}
	}
	catch(boost::thread_interrupted&)
	{
	}
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
	double percent = (dynamixelValue & 0x3FF) * 0.1 * (dynamixelValue & 0x400 ? -1 : 1);
	return percent;
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