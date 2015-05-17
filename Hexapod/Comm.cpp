#include "Comm.h"
#include "Instructions.h"
#include "Point.h"
#include "Protocol.h"

const float ANGLE_FACTOR = 1000.0;

Protocol protocol;

Comm::Comm(Robot* r) : robot(r), _nbRegisteredParameters(0), _nbRegisteredGraphs(0)
{
}

uint8_t* Comm::readUInt8(uint8_t* data, uint8_t& value)
{
	value = data[0];
	return data + 1;
}

uint8_t* Comm::readUInt16(uint8_t* data, uint16_t& value)
{
	value = ((uint16_t)(data[0] << 8)) + (uint16_t)(data[1]);
	return data + 2;
}

uint8_t* Comm::readUInt32(uint8_t* data, uint32_t& value)
{
	value = ((uint32_t)(data[0] << 24)) + ((uint32_t)(data[1] << 16)) + ((uint32_t)(data[2] << 8)) + (uint32_t)data[3];
	return data + 4;
}

uint8_t* Comm::readInt8(uint8_t* data, int8_t& value)
{
	return readUInt8(data, (uint8_t&)value);
}

uint8_t* Comm::readInt16(uint8_t* data, int16_t& value)
{
	return readUInt16(data, (uint16_t&)value);
}

uint8_t* Comm::readInt32(uint8_t* data, int32_t& value)
{
	return readUInt32(data, (uint32_t&)value);
}

uint8_t* Comm::readFloat(uint8_t* data, float& value)
{
	uint32_t iValue = 0;
	uint8_t* res = readUInt32(data, iValue);
	value = *((float*)&iValue);
	return res;
}

uint8_t* Comm::writeInt8(uint8_t* data, uint8_t value)
{
	data[0] = value;
	return data + 1;
}

uint8_t* Comm::writeInt16(uint8_t* data, uint16_t value)
{
	data[0] = (uint8_t)(value >> 8);
	data[1] = (uint8_t)(value % 256);
	return data + 2;
}

uint8_t* Comm::writeInt32(uint8_t* data, uint32_t value)
{
	data[0] = (uint8_t)(value >> 24);
	data[1] = (uint8_t)((value >> 16) % 256);
	data[2] = (uint8_t)((value >> 8) % 256);
	data[3] = (uint8_t)(value % 256);
	return data + 4;
}

uint8_t* Comm::writeFloat(uint8_t* data, float value)
{
	uint32_t iValue = *((uint32_t*)&value);
	uint8_t* res = writeInt32(data, iValue);
	return res;
}

uint8_t *Comm::writeString(uint8_t *data, const String &str)
{
	uint8_t len = str.length() + 1;
	data = writeInt8(data, len);

	str.toCharArray((char*)data, len);

	return data + len + 1;
}

void Comm::sendParameterNames()
{
	String names;

	for(int i = 0; i < _nbRegisteredParameters; ++i)
	{
		if (i > 0)
			names += ";;";

		names += _parameters[i].name;
	}

	int len = names.length() + 1;
	char data[len];
	names.toCharArray(data, len);

	protocol.sendMessage(INSTR_PARAMETERS_NAMES, len, (uint8_t*)data);
}

void Comm::sendParameters()
{
	uint8_t data[_nbRegisteredParameters * 4 + 1];
	uint8_t* dataPtr = &(data[0]);

	dataPtr = writeInt8(dataPtr, _nbRegisteredParameters);
	for(int i = 0; i < _nbRegisteredParameters; ++i)
	{
		float value = 0;
		Parameter& p = _parameters[i];
		if (p.floatValue)
			value = *p.floatValue;
		else if (p.intValue)
			value = *p.intValue;

		dataPtr = writeFloat(dataPtr, value);
	}

	protocol.sendMessage(INSTR_PARAMETERS, _nbRegisteredParameters * 4 + 1, data);
}

void Comm::sendAR(uint8_t instruction, bool ok)
{
	uint8_t data[2];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt8(dataPtr, instruction);
	dataPtr = writeInt8(dataPtr, ok ? 1 : 0);

	protocol.sendMessage(INSTR_AR, 2, data);
}

void Comm::sendSonars(int ag, int ad, int rg, int rd)
{
	uint8_t data[4];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt8(dataPtr, ag < 255 ? ag : 255);
	dataPtr = writeInt8(dataPtr, ad < 255 ? ad : 255);
	dataPtr = writeInt8(dataPtr, rg < 255 ? rg : 255);
	dataPtr = writeInt8(dataPtr, rd < 255 ? rd : 255);

	protocol.sendMessage(INSTR_SEND_SONARS, 4, data);
}

void Comm::sendEvent(uint8_t event, uint8_t parameter)
{
	uint8_t data[2];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt8(dataPtr, event);
	dataPtr = writeInt8(dataPtr, parameter);

	protocol.sendMessage(INSTR_EVENT, 2, data);
}

void Comm::sendSensorEvent(uint8_t sensorType, uint8_t sensorNo, uint8_t sensorState)
{
	uint8_t data[3];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt8(dataPtr, sensorType);
	dataPtr = writeInt8(dataPtr, sensorNo);
	dataPtr = writeInt8(dataPtr, sensorState);

	protocol.sendMessage(INSTR_SENSOR_EVENT, 3, data);
}

void Comm::sendPosition()
{
	short orientation = robot->position.theta * ANGLE_FACTOR;

	uint8_t data[6];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt16(dataPtr, (short)robot->position.x);
	dataPtr = writeInt16(dataPtr, (short)robot->position.y);
	dataPtr = writeInt16(dataPtr, (short)orientation);

	protocol.sendMessage(INSTR_COORD, 6, data);
}

void Comm::sendConsigne()
{
	float orientation = robot->pointSuivant.theta;
	orientation *= ANGLE_FACTOR;

	uint8_t data[6];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt16(dataPtr, robot->pointSuivant.x);
	dataPtr = writeInt16(dataPtr, robot->pointSuivant.y);
	dataPtr = writeInt16(dataPtr, orientation);

	protocol.sendMessage(INSTR_CONSIGNE, 6, data);
}

void Comm::sendGo(bool isBlue)
{
	uint8_t data[1];
	uint8_t* dataPtr = &(data[0]);
	dataPtr = writeInt8(dataPtr, isBlue ? 1 : 0);

	protocol.sendMessage(INSTR_GO, 1, data);
}

void Comm::sendInit()
{
	protocol.sendMessage(INSTR_INIT_DONE, 0, NULL);
}
void Comm::sendLog(const String& text)
{
	int len = text.length() + 1;
	char data[len];
	text.toCharArray(data, len);

	protocol.sendMessage(INSTR_LOG, len, (uint8_t*)data);
}

bool Comm::process_message(uint8_t data[], uint8_t instruction, uint8_t length)
{
	bool ok = false;

	if (instruction == INSTR_DEST_ADD && length == 10)
	{
		//add a point
		Point p;
		int16_t x, y, theta;


		data = readInt16(data, x);
		data = readInt16(data, y);
		data = readInt16(data, theta);

		p.x = x;
		p.y = y;
		p.theta = theta / ANGLE_FACTOR;

		robot->ajoutPoint(p);
		ok = true;
	}
	else if (instruction == INSTR_FLUSH)
	{
		//stop the robot and flush the remaining list of point
		robot->stop();
		ok = true;
	}
	else if (instruction == INSTR_SET_POS && length == 6)
	{
		//set the start point
		int16_t x, y, thetaInt;

		data = readInt16(data, x);
		data = readInt16(data, y);
		data = readInt16(data, thetaInt);
		float theta = (float) thetaInt;
		theta /= ANGLE_FACTOR;

		Point p;
		p.x = x;
		p.y = y;
		p.theta = theta;

		robot->teleport(p);
		ok = true;
	}
	else if (instruction == INSTR_SET_PARAMETERS)
	{
		uint8_t nbParameters = 0;
		data = readUInt8(data, nbParameters);
		for(int i = 0; i < nbParameters && i < _nbRegisteredParameters; ++i)
		{
			float value;
			data = readFloat(data, value);

			Parameter& p = _parameters[i];
			if (p.floatValue)
			{
				if (p.floatSetter)
					p.floatSetter(value);
				else
					*p.floatValue = value;
			}
			else if (p.intValue)
			{
				int iValue = (int)value;
				if (p.intSetter)
					p.intSetter(iValue);
				else
					*p.intValue = iValue;
			}

		}

		ok = true;
	}
	else if (instruction == INSTR_ASK_PARAMETERS)
	{
		sendParameters();
		sendParameterNames(); //always include names, could be refined
		ok = true;
	}
	else if (instruction == INSTR_ASK_GRAPHS)
	{
		sendRegisteredGraphs();
		ok = true;
	}
	else if (instruction == INSTR_PING)
	{
		ok = true;
	}
	else if (instruction == INSTR_AR)
	{
		ok = true;
	}

	return ok;
}

void Comm::commRead()
{
	if (protocol.read())
	{
		bool ok = process_message(protocol.getData(), protocol.getInstruction(), protocol.getLength());

		if (protocol.getInstruction() == INSTR_PING)
			sendAR(INSTR_PING, ok);
	}
}

void Comm::registerParameter(float* value, const String& name, void (*setter)(float))
{
	if (_nbRegisteredParameters >= MAX_PARAMETERS)
		return;

	_parameters[_nbRegisteredParameters] =  Parameter(value, name, setter);
	++_nbRegisteredParameters;
}

void Comm::registerParameter(int* value, const String& name, void (*setter)(int))
{
	if (_nbRegisteredParameters >= MAX_PARAMETERS)
		return;

	_parameters[_nbRegisteredParameters] =  Parameter(value, name, setter);
	++_nbRegisteredParameters;
}

void Comm::registerGraph(int graphId, GraphType type, const String &name, String parameterNames[])
{
	Graph& g = _graphs[_nbRegisteredGraphs];
	g.id = graphId;
	g.type = type;
	g.name = name;

	int nbParameters = sizeof(parameterNames)/sizeof(parameterNames[0]);
	for(int i = 0; i < nbParameters; ++i)
		g.parameterNames[i] = parameterNames[i];
	g.parameterCount = nbParameters;

	++_nbRegisteredGraphs;
}

void Comm::sendRegisteredGraphs()
{
	for(int i = 0; i < _nbRegisteredGraphs; ++i)
	{
		Graph& g = _graphs[i];

		uint8_t data[250];
		uint8_t* dataPtr = &(data[0]);
		dataPtr = writeInt8(dataPtr, g.id);
		dataPtr = writeInt8(dataPtr, g.type);

		uint8_t len = 2;
		dataPtr = writeString(dataPtr, g.name); len += g.name.length() + 2;
		dataPtr = writeInt8(dataPtr, g.parameterCount); len += 1;

		for(int i = 0; i < g.parameterCount; ++i)
		{
			const String& paramName = g.parameterNames[i];
			dataPtr = writeString(dataPtr, paramName);
			len += paramName.length() + 2;
		}

		protocol.sendMessage(INSTR_REGISTER_GRAPH, len, dataPtr);
	}
}

void Comm::sendGraphValues(int graphId, float values[], int nbParameters)
{
	uint8_t len = nbParameters * 4 + 1;
	uint8_t data[len];
	uint8_t* dataPtr = &(data[0]);

	dataPtr = writeInt8(dataPtr, graphId);
	for(int i = 0; i < nbParameters; ++i)
	{
		dataPtr = writeFloat(dataPtr, values[i]);
	}

	protocol.sendMessage(INSTR_GRAPH_VALUES, len, dataPtr);
}

void Comm::sendGraphSingleValue(int graphId, int paramId, float value)
{
	uint8_t len = 4 + 2;
	uint8_t data[len];
	uint8_t* dataPtr = &(data[0]);

	dataPtr = writeInt8(dataPtr, graphId);
	dataPtr = writeInt8(dataPtr, paramId);
	dataPtr = writeFloat(dataPtr, value);

	protocol.sendMessage(INSTR_GRAPH_SINGLE_VALUE, len, dataPtr);
}
