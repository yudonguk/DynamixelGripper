#ifndef __SERIAL_COMMUNICATOR_H__
#define __SERIAL_COMMUNICATOR_H__

#include <string>

#include <boost/current_function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include <device/Uart.h>

#define PROPERTY_PORT_NAME		"PortName"
#define PROPERTY_TIME_OUT		"TimeOut"
#define PROPERTY_BAUD_RATE		"BaudRate"
#define PROPERTY_DATA_BITS		"DataBits"
#define PROPERTY_STOP_BITS		"StopBits"
#define PROPERTY_PARITY			"Parity"
#define PROPERTY_FLOW_CONTROL	"FlowControl"

#define DEBUG_PRINT(__MESSAGE__) std::clog << __MESSAGE__ \
	<< " << " << BOOST_CURRENT_FUNCTION  << "(" << __LINE__ << ")" << std::endl

class SerialCommunicator : public Uart
{
private:
	struct SerialPortProperty  
	{
		std::string portName;
		unsigned int timeOut;
		unsigned int baudRate;
		unsigned int dataBits;
		unsigned int stopBits;
		unsigned int parity;
		unsigned int flowControl;

		SerialPortProperty()
			: portName(), timeOut(), baudRate(), dataBits(), stopBits()
			, parity(), flowControl()
		{		
		}
	};

	class BusLockGuard : private boost::noncopyable
	{
	public:
		BusLockGuard(Bus& bus)
			: mBus(bus)
		{
			mBus.Lock();
		}

		~BusLockGuard()
		{
			mBus.Unlock();
		}

	public:
		void Lock()
		{
			mBus.Lock();
		}

		void Unlock()
		{
			mBus.Unlock();
		}

	private:
		Bus& mBus;
	};

public:
	SerialCommunicator()
		: mSerialPort(mIoService), mTimer(mIoService)
	{
	}

	virtual ~SerialCommunicator()
	{
		Finalize();
	}

public:
	virtual int Initialize(Property parameter) 
	{
		BusLockGuard guard(*this);
		return SetParameter(parameter); 
	}

	virtual int Finalize()
	{
		BusLockGuard guard(*this);
		return Disable(); 
	}

	virtual int Enable() 
	{
		BusLockGuard guard(*this);
		return API_SUCCESS; 
	}

	virtual int Disable() 
	{ 
		BusLockGuard guard(*this);

		if (!mSerialPort.is_open())
			return API_SUCCESS;

		boost::system::error_code error; 
		if(mSerialPort.close(error))
		{
			DEBUG_PRINT(error.message());
		}

		return API_SUCCESS; 
	}

	virtual int SetParameter(Property parameter) 
	{		
		SerialPortProperty property;

		std::clog << "SerialCommunicator Property Setting" << std::endl;

		if(!(Getter(property.portName, parameter, PROPERTY_PORT_NAME)
			&& Getter(property.timeOut, parameter, PROPERTY_TIME_OUT)
			&& Getter(property.baudRate, parameter, PROPERTY_BAUD_RATE)
			&& Getter(property.dataBits, parameter, PROPERTY_DATA_BITS)
			&& Getter(property.stopBits, parameter, PROPERTY_STOP_BITS)
			&& Getter(property.parity, parameter, PROPERTY_PARITY)
			&& Getter(property.flowControl, parameter, PROPERTY_FLOW_CONTROL)
			))
			return API_ERROR;

		BusLockGuard guard(*this);

		Disable();

		mProperty = property;

		using boost::asio::serial_port_base;

		boost::system::error_code error; 
		if(mSerialPort.open(mProperty.portName, error))
		{
			DEBUG_PRINT(error.message());
			return API_ERROR;
		}

		if(!(SetSerialOption(mSerialPort, serial_port_base::baud_rate(property.baudRate))
			&& SetSerialOption(mSerialPort, serial_port_base::character_size(property.dataBits))
			&& SetSerialOption(mSerialPort, serial_port_base::stop_bits(static_cast<serial_port_base::stop_bits::type>(property.stopBits)))
			&& SetSerialOption(mSerialPort, serial_port_base::parity(static_cast<serial_port_base::parity::type>(property.parity)))
			&& SetSerialOption(mSerialPort, serial_port_base::flow_control(static_cast<serial_port_base::flow_control::type>(property.flowControl)))
			))
			return API_ERROR;

		return API_SUCCESS;
	}

	virtual int GetParameter(Property &parameter) 
	{
		Lock();
		SerialPortProperty property = mProperty;
		Unlock();

		if(!(Setter(parameter, PROPERTY_PORT_NAME, property.baudRate)
			&& Setter(parameter, PROPERTY_TIME_OUT, property.timeOut)
			&& Setter(parameter, PROPERTY_BAUD_RATE, property.baudRate)
			&& Setter(parameter, PROPERTY_DATA_BITS, property.dataBits)
			&& Setter(parameter, PROPERTY_STOP_BITS, property.stopBits)
			&& Setter(parameter, PROPERTY_PARITY, property.parity)
			&& Setter(parameter, PROPERTY_FLOW_CONTROL, property.flowControl)
			))
			return API_ERROR;

		return API_SUCCESS; 
	}

	virtual int OnExecute() 
	{
		return API_NOT_SUPPORTED; 
	}

public:
	virtual int Write(unsigned char *data, int size) 
	{
		BusLockGuard guard(*this);

		mIoService.reset();

		size_t sentSize = -1;

		boost::asio::async_write(mSerialPort, boost::asio::buffer(data, size)
			, boost::bind(&ReadWriteHandler<decltype(mTimer)>, boost::ref(sentSize)
			, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
			, boost::ref(mTimer)));

		mTimer.expires_from_now(boost::posix_time::milliseconds(mProperty.timeOut));
		mTimer.async_wait(boost::bind(&TimeoutHandler<decltype(mSerialPort)>
			, boost::asio::placeholders::error
			, boost::ref(mSerialPort)));

		boost::system::error_code error;
		mIoService.run(error);
		if(error)
			DEBUG_PRINT(error.message());

		if (sentSize < 0)
			return API_ERROR;
		return sentSize;
	}

	virtual int Read(unsigned char *data, int size) 
	{
		BusLockGuard guard(*this);

		mIoService.reset();

		size_t readSize = -1;

		boost::asio::async_read(mSerialPort, boost::asio::buffer(data, size)
			, boost::bind(&ReadWriteHandler<decltype(mTimer)>, boost::ref(readSize)
			, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
			, boost::ref(mTimer)));

		mTimer.expires_from_now(boost::posix_time::milliseconds(mProperty.timeOut));
		mTimer.async_wait(boost::bind(&TimeoutHandler<decltype(mSerialPort)>
			, boost::asio::placeholders::error
			, boost::ref(mSerialPort)));

		boost::system::error_code error;
		mIoService.run(error);
		if(error)
			DEBUG_PRINT(error.message());

		if (readSize < 0)
			return API_ERROR;
		return readSize;
	}

private:
	template<typename Output, typename PropertyName>
	static bool Getter(Output& output, Property& property, const PropertyName& name)
	{
		if (!property.FindName(name))
		{
			DEBUG_PRINT("Can't find " << name);
			return false;
		}
		const std::string& value = property.GetValue(name);

		try
		{
			output = boost::lexical_cast<Output>(value);
		}
		catch(const boost::bad_lexical_cast& e)
		{
			DEBUG_PRINT(e.what());
			return false;
		}

		std::clog << name << " = " << output << std::endl;

		return true;
	}

	template<typename Output, typename PropertyName>
	static bool Setter(Property& property, const PropertyName& name, const Output& input)
	{
		try
		{
			property.SetValue(name, boost::lexical_cast<std::string>(input));
		}
		catch(const boost::bad_lexical_cast& e)
		{
			DEBUG_PRINT(e.what());
			return false;
		}
		return true;
	}

	template<typename SettableSerialPortOption>
	static bool SetSerialOption(boost::asio::serial_port& serial, const SettableSerialPortOption& option)
	{
		boost::system::error_code error;
		if (serial.set_option(option, error))
		{
			DEBUG_PRINT(error.message());
			return false;
		}

		return true;
	}

	template<typename CancelOperation>
	static void ReadWriteHandler(size_t& result, const boost::system::error_code& error, size_t bytesTransferred, CancelOperation& operation)
	{
		result = bytesTransferred;

		switch (error.value())
		{
		case boost::system::errc::success: break;
		case boost::asio::error::operation_aborted: return; break;				
		default: 
			result = -1;
			DEBUG_PRINT(error.message()); 
			return; break;
		}

		boost::system::error_code cancelError;
		operation.cancel(cancelError);
		if(cancelError)
			DEBUG_PRINT(cancelError.message());
	}

	template<typename CancelOperation>
	static void TimeoutHandler(const boost::system::error_code& error, CancelOperation& operation)
	{
		switch (error.value())
		{
		case boost::system::errc::success: break;
		case boost::asio::error::operation_aborted: return; break;				
		default: DEBUG_PRINT(error.message()); return; break;
		}

		boost::system::error_code cancelError;
		operation.cancel(cancelError);
		if(cancelError)
			DEBUG_PRINT(cancelError.message());
	}

private:
	SerialPortProperty mProperty;

	boost::asio::io_service mIoService;
	boost::asio::serial_port mSerialPort;
	boost::asio::deadline_timer mTimer;
};

#undef PROPERTY_PORT_NAME
#undef PROPERTY_TIME_OUT
#undef PROPERTY_BAUD_RATE
#undef PROPERTY_DATA_BITS
#undef PROPERTY_STOP_BITS
#undef PROPERTY_PARITY
#undef PROPERTY_FLOW_CONTROL
#undef DEBUG_PRINT

#endif // !__SERIAL_COMMUNICATOR_H__
