#include "DynamixelGripper.h"

#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/locks.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

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
#define HOME_POSITION			"HomePosition"
#define MAXIMUM_LOAD			"MaximumLoad"
#define LOAD_CONTROL_P_GAIN		"LoadControlPGain"
#define LOAD_CONTROL_I_GAIN		"LoadControlIGain"

DynamixelGripper::DynamixelGripper()
	: mpUart(NULL), mIsGripped(false)
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

	_status = DEVICE_READY;
	return API_SUCCESS;
}

int DynamixelGripper::Finalize()
{
	if (_status == DEVICE_ACTIVE)
		Disable();

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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

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
	lock.unlock();

	StopGripping();
	RunHoming();

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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

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
	if (parameter.FindName(JOINT_COUNT) == false)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't find Size<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}
	const size_t jointCount = boost::lexical_cast<size_t>(parameter.GetValue(JOINT_COUNT));

	PrintMessage("\r\nDynamixelGripper Property Setting\r\n");

	std::vector<boost::shared_ptr<DynamixelProperty>> dynamixelProperties;
	DynamixelGroup dynamixelGroup;

	char buff[100] = {0, };
	for (size_t i = 0;  i < jointCount; i++)
	{
		boost::shared_ptr<DynamixelProperty> pProperty = boost::make_shared<DynamixelProperty>();

		//CounterclockwiseMode
		sprintf(buff, "%s%d", COUNTERCLOCKWISE_MODE, i);
		if (parameter.FindName(buff)) 
			pProperty->isCounterclockwiseMode
			= boost::iequals(parameter.GetValue(buff), "true") ? true : false;
		PrintMessage("%s : %s \r\n", buff, pProperty->isCounterclockwiseMode ? "true" : "false");

		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID, i);
		if (parameter.FindName(buff) == false) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pProperty->id = boost::lexical_cast<int>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pProperty->id);

		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE, i);
		if (parameter.FindName(buff)) 
			pProperty->complianceMargine
			= boost::lexical_cast<int>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pProperty->complianceMargine);

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE, i);
		if (parameter.FindName(buff)) 
			pProperty->compliacneSlope
			= boost::lexical_cast<int>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pProperty->compliacneSlope);

		//PositionResolution
		sprintf(buff, "%s%d", POSITION_RESOLUTION, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pProperty->positionResolution
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->positionResolution);

		//PositionOffset
		sprintf(buff, "%s%d", POSITION_OFFSET, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pProperty->positionOffset
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->positionOffset);

		//MaximumPower
		sprintf(buff, "%s%d", MAXIMUM_POWER, i);
		if (parameter.FindName(buff))
			pProperty->maximumPower
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximumPower);

		//MaximumVelocity
		sprintf(buff, "%s%d", MAXIMUM_VELOCITY, i);
		if (parameter.FindName(buff)) 
			pProperty->maximuVelocity
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximuVelocity);

		//MinimumPositionLimit
		sprintf(buff, "%s%d", MINIMUM_POSITION_LIMIT, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		} 
		pProperty->minimumPositionLimit
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->minimumPositionLimit);

		//MaximumPositionLimit
		sprintf(buff, "%s%d", MAXIMUM_POSITION_LIMIT, i);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			return false;
		}
		pProperty->maximumPositionLimit
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximumPositionLimit);

		//HomePosition
		sprintf(buff, "%s%d", HOME_POSITION, i);
		if (parameter.FindName(buff)) 
			pProperty->homePosition
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximumPositionLimit);

		PrintMessage("\r\n");

		if(pProperty->id == DummyDynamixelUart::DUMMY_ID)
			pProperty->pDynamixel = boost::make_shared<DummyDynamixelUart>();
		else
			pProperty->pDynamixel = boost::make_shared<DynamixelUART>((Uart*)NULL, pProperty->id);

		dynamixelProperties.push_back(pProperty);
		dynamixelGroup.push_back(pProperty->pDynamixel);
	}

	PIControl gripperLoadPIControl;
	{
		boost::shared_ptr<GripperDynamixelProperty> pProperty = boost::make_shared<GripperDynamixelProperty>();

		bool isEnoughGripperProperty = true;

		//CounterclockwiseMode
		sprintf(buff, "Gripper%s", COUNTERCLOCKWISE_MODE);
		if (parameter.FindName(buff)) 
			pProperty->isCounterclockwiseMode 
			= boost::iequals(parameter.GetValue(buff), "true") ? true : false;;
		PrintMessage("%s : %s \r\n", buff, pProperty->isCounterclockwiseMode ? "true" : "false");

		//DynamixelID
		sprintf(buff, "Gripper%s", DYNAMIXEL_ID);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty = false;
		}
		else
		{
			pProperty->id = boost::lexical_cast<int>(parameter.GetValue(buff));
			PrintMessage("%s : %d \r\n", buff, pProperty->id);	
		}

		//ComplianceMargine
		sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE);
		if (parameter.FindName(buff)) 
			pProperty->complianceMargine
			= boost::lexical_cast<int>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pProperty->complianceMargine);	

		//ComplianceSlope
		sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE);
		if (parameter.FindName(buff)) 
			pProperty->compliacneSlope
			= boost::lexical_cast<int>(parameter.GetValue(buff));
		PrintMessage("%s : %d \r\n", buff, pProperty->compliacneSlope);	

		//PositionResolution
		sprintf(buff, "Gripper%s", POSITION_RESOLUTION);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pProperty->positionResolution
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pProperty->positionResolution);	
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
			pProperty->positionOffset
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pProperty->positionOffset);	
		}

		//MaximumPower
		sprintf(buff, "Gripper%s", MAXIMUM_POWER);
		if (parameter.FindName(buff))
			pProperty->maximumPower
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximumPower);	

		//MaximumVelocity
		sprintf(buff, "Gripper%s", MAXIMUM_VELOCITY);
		if (parameter.FindName(buff)) 
			pProperty->maximuVelocity
			= boost::lexical_cast<double>(parameter.GetValue(buff));
		PrintMessage("%s : %lf \r\n", buff, pProperty->maximuVelocity);	

		//MinimumPositionLimit
		sprintf(buff, "Gripper%s", MINIMUM_POSITION_LIMIT);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pProperty->minimumPositionLimit
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pProperty->minimumPositionLimit);	
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
			pProperty->maximumPositionLimit
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pProperty->maximumPositionLimit);	
		}

		//MaximumLoad
		sprintf(buff, "Gripper%s", MAXIMUM_LOAD);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			pProperty->maximumLoad
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, pProperty->maximumLoad);	
		}

		//LoadControlPGain
		sprintf(buff, "Gripper%s", LOAD_CONTROL_P_GAIN);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			gripperLoadPIControl.kp
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, gripperLoadPIControl.kp);	
		}

		//LoadControlIGain
		sprintf(buff, "Gripper%s", LOAD_CONTROL_I_GAIN);
		if (!parameter.FindName(buff)) 
		{
			PrintMessage("Error : DynamixelManipulator::Setting()->Can't find %s<< %s(%d)\r\n", buff, __FILE__, __LINE__);
			isEnoughGripperProperty =  false;
		}
		else
		{
			gripperLoadPIControl.ki
				= boost::lexical_cast<double>(parameter.GetValue(buff));
			PrintMessage("%s : %lf \r\n", buff, gripperLoadPIControl.ki);	
		}

		if (isEnoughGripperProperty)
		{	
			pProperty->pDynamixel = boost::make_shared<DynamixelUART>((Uart*)NULL, pProperty->id);
		}
		else
		{
			pProperty->id = DummyDynamixelUart::DUMMY_ID;
			pProperty->pDynamixel =  boost::make_shared<DummyDynamixelUart>();

			PrintMessage("DynamixelGripper will operate Manipulator. \r\n");
			PrintMessage("Because property of related gripper is not enough. \r\n");			
		}

		PrintMessage("\r\n");	

		dynamixelProperties.push_back(pProperty);
		dynamixelGroup.push_back(pProperty->pDynamixel);
	}

	boost::unique_lock<boost::shared_mutex> lock(mJointStateMutex);

	if (mpUart != NULL)
		mpUart->Finalize();

	mpUart.reset(new SerialCommunicator);

	if(mpUart->Initialize(parameter) != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't Initialize UART<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}
	
	if(mpUart->Enable() != API_SUCCESS)
	{
		PrintMessage("Error : DynamixelManipulator::Setting()->Can't Enable UART<< %s(%d)\r\n", __FILE__, __LINE__);
		return false;
	}

	dynamixelGroup.SetUart(mpUart.get());
	for (size_t i = 0, end = dynamixelProperties.size(); i < end; i++)
	{
		dynamixelProperties[i]->pDynamixel->SetUart(mpUart.get());
	}
	
	mDynamixelProperties = boost::move(dynamixelProperties);
	mDynamixelGroup = boost::move(dynamixelGroup);

	mJointPosition = boost::move(std::vector<double>(mDynamixelProperties.size()));
	mDesiredJointPosition = boost::move(std::vector<double>(mDynamixelProperties.size()));

	mGripperLoadPIControl = boost::move(gripperLoadPIControl);

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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

	if (mpUart->GetParameter(parameter) != API_SUCCESS)
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
		parameter.SetValue(buff, property.isCounterclockwiseMode ? "true" : "false");

		//DynamixelID
		sprintf(buff, "%s%d", DYNAMIXEL_ID, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.id)));

		//ComplianceMargine
		sprintf(buff, "%s%d", COMPLIANCE_MARGINE, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.complianceMargine)));

		//ComplianceSlope
		sprintf(buff, "%s%d", COMPLIANCE_SLOPE, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.compliacneSlope)));

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

		//HomePosition
		sprintf(buff, "%s%d", HOME_POSITION, i);
		parameter.SetValue(buff, boost::lexical_cast<std::string>(property.homePosition));
	}

	{
		GripperDynamixelProperty& property 
			= static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());

		if (property.id != DummyDynamixelUart::DUMMY_ID)
		{
			//CounterclockwiseMode
			sprintf(buff, "Gripper%s", COUNTERCLOCKWISE_MODE);
			parameter.SetValue(buff, property.isCounterclockwiseMode ? "true" : "false");

			//DynamixelID
			sprintf(buff, "Gripper%s", DYNAMIXEL_ID);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.id)));

			//ComplianceMargine
			sprintf(buff, "Gripper%s", COMPLIANCE_MARGINE);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.complianceMargine)));

			//ComplianceSlope
			sprintf(buff, "Gripper%s", COMPLIANCE_SLOPE);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(int(property.compliacneSlope)));

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

			//LoadControlPGain
			sprintf(buff, "Gripper%s", LOAD_CONTROL_P_GAIN);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(mGripperLoadPIControl.kp));

			//LoadControlIGain
			sprintf(buff, "Gripper%s", LOAD_CONTROL_I_GAIN);
			parameter.SetValue(buff, boost::lexical_cast<std::string>(mGripperLoadPIControl.ki));
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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

	//모든 위치를 0으로 
	std::vector<double> position(mDynamixelProperties.size() - 1);
	std::vector<unsigned long> time(position.size());

	for (size_t i = 0, end = position.size(); i < end; i++)
	{
		position[i] = mDynamixelProperties[i]->homePosition;
	}

	lock.unlock();

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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

	mpUart->Lock();
	if (mDynamixelGroup.SetTorqueEnable(false) == false)
	{
		PrintMessage("Error : DynamixelManipulator::EmergencyStop()->Can't EmergencyStop Dynamixel.<< %s(%d)\r\n", __FILE__, __LINE__);
		mpUart->Unlock();
		return API_ERROR;
	}
	mpUart->Unlock();
	return API_SUCCESS;
}

int DynamixelGripper::SetPosition( vector<double> position, vector<unsigned long> time )
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::SetPosition()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	boost::unique_lock<boost::shared_mutex> lock(mJointStateMutex);

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

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

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

	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mJointStateMutex);

	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());

	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
	mGripperCommand = START_GRIPPING;	
	
	return API_SUCCESS;
}

int DynamixelGripper::StopGripping()
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::IsGripped()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mJointStateMutex);

	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
	mGripperCommand = STOP_GRIPPING;
	
	return API_SUCCESS;
}

int DynamixelGripper::IsGripped(bool &isGripped)
{
	if(_status == DEVICE_CREATED)
	{
		PrintMessage("Error : DynamixelManipulator::IsGripped()->Precondition not met<< %s(%d)\r\n", __FILE__, __LINE__);
		return API_ERROR;
	}

	boost::shared_lock<boost::shared_mutex> lock(mJointStateMutex);

	GripperDynamixelProperty& property = static_cast<GripperDynamixelProperty&>(**mDynamixelProperties.rbegin());
	if (property.id == DummyDynamixelUart::DUMMY_ID)
		return API_NOT_SUPPORTED;

	if (mGripperCommand == START_GRIPPING)
	{		
		// 그리퍼 조인트에 걸리는 부하가 목표 부하의 80% 이상 일경우
		if ((property.maximumLoad < 0.0 == mGripperJointLoad < 0.0)
			&& std::abs(mGripperJointLoad) > std::abs(property.maximumLoad * 0.8))
			return 1;
	}
	
	return 0;
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
	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mJointStateMutex);

	std::vector<unsigned short> rawJointPosition;
	std::vector<double> jointPosition(mJointPosition.size());
	unsigned short rawGripperJointLoad = 0;

	DynamixelProperty& gripperProperty = **mDynamixelProperties.rbegin();

	mpUart->Lock();
	size_t positionResult = mDynamixelGroup.GetPresentPosition(rawJointPosition);
	bool resultOfGettingGripperLoad = gripperProperty.pDynamixel->GetPresentLoad(rawGripperJointLoad);
	mpUart->Unlock();

	for (size_t i = 0, end = mDynamixelProperties.size(); i < end; i++)
	{
		if (!(positionResult & (1 << i)))
		{
			// 조인트의 위치를 가져오지 못할 경우, 이전 조인트 위치를 적용한다.
			jointPosition[i] = mJointPosition[i];
			continue;
		}

		DynamixelProperty& property = *mDynamixelProperties[i];

		// 단위계 변환
		jointPosition[i] = (property.isCounterclockwiseMode ? 1.0 : -1.0)
			* ConvertPositionUnitToDegree(rawJointPosition[i]
		, property.positionOffset, property.positionResolution);
	}

	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
	mJointPosition = boost::move(jointPosition);
	// 그리퍼 조인트의 하중을 얻어왔을 경우에만 갱신
	if(resultOfGettingGripperLoad)
		mGripperJointLoad = (gripperProperty.isCounterclockwiseMode ? 1.0 : -1.0)
		* ConvertLoadUnitToPercent(rawGripperJointLoad);
}

void DynamixelGripper::ControlJoint()
{
	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mJointStateMutex);
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

		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		// 다음 계산을 위해 저장
		mGripperLoadPIControl.time = now;
		mGripperLoadPIControl.error = loadError;
		mGripperLoadPIControl.manipulatedValue = manipulatedValue;

		*mDesiredJointPosition.rbegin() = manipulatedValue;
	}
	else
	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);

		mGripperLoadPIControl.time = boost::chrono::high_resolution_clock::now();
		mGripperLoadPIControl.error = 0.0;
		mGripperLoadPIControl.manipulatedValue = gripperProperty.minimumPositionLimit;

		*mDesiredJointPosition.rbegin() = gripperProperty.minimumPositionLimit;
	}

	//단위 변환
	vector<unsigned short> rawJointPosition(mDesiredJointPosition.size());
	for (size_t i = 0, end = rawJointPosition.size();  i < end; i++)
	{
		DynamixelProperty& property = *mDynamixelProperties[i];

		rawJointPosition[i] = ConvertPositionUnitToDynamixel(
			(property.isCounterclockwiseMode ? 1.0 : -1.0) * mDesiredJointPosition[i]
		, property.positionOffset, property.positionResolution);
	}

	// 조인트 위치 설정
	mpUart->Lock();
	mDynamixelGroup.SetGoalPosition(rawJointPosition);
	mpUart->Unlock();
}

#ifdef _WIN32
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
	return new DynamixelGripper();
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
#undef HOME_POSITION
#undef MAXIMUM_LOAD
#undef LOAD_CONTROL_P_GAIN
#undef LOAD_CONTROL_I_GAIN
