#ifndef __DYNAMIXEL_MANIPULATOR_H__
#define __DYNAMIXEL_MANIPULATOR_H__

#include <boost/thread/shared_mutex.hpp>

#include <device/Gripper.h>

#include "DynamixelUART.h"
#include "DynamixelGroup.h"

class DynamixelGripper : public Gripper
{
public:
	enum GripperCommand
	{
		START_GRIPPING = 1, STOP_GRIPPING = START_GRIPPING + 1
	};

protected:
	struct DynamixelProperty
	{
		DynamixelProperty()
			: id(0), maximumPower(0.0), maximuVelocity(0.0)
			, complianceMargine(0), compliacneSlope(0)
			, minimumPositionLimit(0.0), maximumPositionLimit(0.0)
			, positionResolution(0.0), positionOffset(0.0)
		{}

		unsigned char id;
		unsigned char complianceMargine;
		unsigned char compliacneSlope;
		double positionResolution;
		double positionOffset;
		double maximumPower;
		double maximuVelocity;
		double minimumPositionLimit;
		double maximumPositionLimit;
	};

	struct GripperDynamixelProperty : public DynamixelProperty
	{
		GripperDynamixelProperty()
			: maximumLoad(0.0)
		{}

		double maximumLoad;
	};

public:
	DynamixelGripper();
	virtual ~DynamixelGripper();

public:
	virtual int Initialize(Property parameter);
	virtual int Finalize();
	virtual int Enable();
	virtual int Disable();
	virtual int SetParameter(Property parameter);
	virtual int GetParameter(Property &parameter);
	virtual int OnExecute();

public:
	virtual int RunHoming();
	virtual int Stop();
	virtual int EmergencyStop();
	virtual int SetPosition(vector<double> position, vector<unsigned long> time);
	virtual int GetPosition(vector<double>& position);

public:
	virtual int StartGripping();
	virtual int StopGripping();
	virtual int IsGripped(bool &isGripped);

private:
	bool Setting(Property& parameter);
	bool EnableDynamixel(DynamixelUART& dynamixel, const DynamixelProperty& property);

	inline unsigned short ConvertPowerUnitToDynamixel(const double& percent);
	inline unsigned short ConvertPositionUnitToDynamixel(const double& degree, const double& offset, const double& resolution);
	inline unsigned short ConvertVelocityUnitToDynamixel(const double& rpm);

	inline double ConvertPowerUnitToPercent(unsigned short dynamixelValue);
	inline double ConvertPositionUnitToDegree(unsigned short dynamixelValue, const double& offset, const double& resolution);
	inline double ConvertVelocityUnitToRPM(unsigned short dynamixelValue);
	inline double ConvertLoadUnitToPercent(unsigned short dynamixelValue);

private:
	void UpdateJointState();
	void ControlJoint();

private:
	// dynamixelGroup의 마지막 원소는 그리퍼의 조인트를 가르킨다.
	DynamixelGroup dynamixelGroup;
	vector<DynamixelProperty> dynamixelPropertyVector;

	GripperDynamixelProperty gripperProperty;
		
	Uart* uart;
	bool mIsGripped;

	boost::shared_mutex mJointPositionMutex;
	// mJointPosition의 마지막 원소는 그리퍼 조인트의 위치이다.
	std::vector<double> mJointPosition;
	std::vector<double> mDesiredJointPosition;
	double mGripperJointLoad;
	GripperCommand mGripperCommand;
};

#endif //__DYNAMIXEL_MANIPULATOR_H__