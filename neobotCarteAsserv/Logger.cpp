#include "Logger.h"

#define SerialDEBUG Serial

Logger::Logger(Comm *comm, bool enableDebug, bool useComm) : _comm(comm), _enable(enableDebug), _useComm(useComm)
{
}

void Logger::useComm(bool use)
{
    _useComm = use;
}

void Logger::print(const String &string)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += string;
		}
		else
		{
			SerialDEBUG.print(string);
		};
}

void Logger::println(const String &string)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += string;
			_comm->sendLog(_partialString);
			_partialString = String();
		}
		else
		{
			SerialDEBUG.println(string);
		};
}

void Logger::print(int value)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += String(value);
		}
		else
		{
			SerialDEBUG.print(value);
		};
}

void Logger::println(int value)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += String(value);
			_comm->sendLog(_partialString);
			_partialString = String();
		}
		else
		{
			SerialDEBUG.println(value);
		};
}

void Logger::print(double value)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += String(value);
		}
		else
		{
			SerialDEBUG.print(value);
		};
	}

void Logger::println(double value)
{
	if (_enable)
		if (_useComm)
		{
			_partialString += String(value);
			_comm->sendLog(_partialString);
			_partialString = String();
		}
		else
		{
			SerialDEBUG.println(value);
		};
}
