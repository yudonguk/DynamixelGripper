#ifndef __DYNAMIXEL_MANIPULATOR_H__
#define __DYNAMIXEL_MANIPULATOR_H__

#include <device/Manipulator.h>
#include <device/Gripper.h>
#include <device/ServoActuator.h>
#include <oprostypes.h>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

#include "DynamixelUART.h"
#include "DynamixelGroup.h"
#include "DummyDynamixelUART.h"

#include "MessageQueue.h"

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
		DynamixelProperty(boost::shared_ptr<DynamixelUART>& pDynamixel_ = boost::shared_ptr<DynamixelUART>())
			: pDynamixel(pDynamixel_), isCounterclockwiseMode(true)
			, id(DummyDynamixelUart::DUMMY_ID), complianceMargine(1)
			, compliacneSlope(32), positionResolution(0.0), positionOffset(0.0)
			, maximumPower(100.0), maximuVelocity(45.0)
			, minimumPositionLimit(-180.0), maximumPositionLimit(180.0)
			, homePosition(0.0)
		{}

		boost::shared_ptr<DynamixelUART> pDynamixel;
		
		bool isCounterclockwiseMode;
		unsigned char id;
		unsigned char complianceMargine;
		unsigned char compliacneSlope;
		double positionResolution;
		double positionOffset;
		double maximumPower;
		double maximuVelocity;
		double minimumPositionLimit;
		double maximumPositionLimit;
		double homePosition;
	};

	struct GripperDynamixelProperty : public DynamixelProperty
	{
		GripperDynamixelProperty(boost::shared_ptr<DynamixelUART>& pDynamixel_ = boost::shared_ptr<DynamixelUART>())
			: DynamixelProperty(pDynamixel_), maximumLoad(0.0)
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

	static unsigned short ConvertPowerUnitToDynamixel(const double& percent);
	static unsigned short ConvertPositionUnitToDynamixel(const double& degree, const double& offset, const double& resolution);
	static unsigned short ConvertVelocityUnitToDynamixel(const double& rpm);

	static double ConvertPowerUnitToPercent(unsigned short dynamixelValue);
	static double ConvertPositionUnitToDegree(unsigned short dynamixelValue, const double& offset, const double& resolution);
	static double ConvertVelocityUnitToRPM(unsigned short dynamixelValue);
	static double ConvertLoadUnitToPercent(unsigned short dynamixelValue);

private:
	void GripperControlThreadHandler();

private:
	// mDynamixelGroup과 mDynamixelProperties의 마지막 원소는 그리퍼의 조인트를 가르킨다.
	DynamixelGroup mDynamixelGroup;
	std::vector<boost::shared_ptr<DynamixelProperty>> mDynamixelProperties;

	MessageQueue<GripperCommand> gripperMessageQueue;
	boost::thread* gripperControlThread;

	boost::scoped_ptr<Uart> pUart;
	bool mIsGripped;

	boost::shared_mutex mJointPositionMutex;
	// mJointPosition의 마지막 원소는 그리퍼 조인트의 위치이다.
	std::vector<double> mJointPosition;
	double mGripperJointLoad;
};

#endif //__DYNAMIXEL_MANIPULATOR_H__