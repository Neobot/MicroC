#ifndef COMM_H
#define COMM_H

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */

#include "Robot.h"

class Logger;

#define MAX_PARAMETERS 25

class Comm
{

public:
  Comm(Robot* r);

  void setLogger(Logger* logger);

  uint8_t* readInt8(uint8_t* data, int8_t& value);
  uint8_t* readInt16(uint8_t* data, int16_t& value);
  uint8_t* readInt32(uint8_t* data, int32_t& value);
  uint8_t* readUInt8(uint8_t* data, uint8_t& value);
  uint8_t* readUInt16(uint8_t* data, uint16_t& value);
  uint8_t* readUInt32(uint8_t* data, uint32_t& value);
  uint8_t* readFloat(uint8_t* data, float& value);
  uint8_t* writeInt8(uint8_t* data, uint8_t value);
  uint8_t* writeInt16(uint8_t* data, uint16_t value);
  uint8_t* writeInt32(uint8_t* data, uint32_t value);
  uint8_t* writeFloat(uint8_t* data, float value);
  uint8_t* writeString(uint8_t* data, const String& str);
  
  void sendParameters();
  void sendParameterNames();
  void sendAR(uint8_t instruction, bool ok);
  void sendSonars(int ag, int ad, int rg, int rd);
  void sendColorSensorsEvents();
  void sendEvent(uint8_t event, uint8_t parameter = 0);
  void sendSensorEvent(uint8_t sensorType, uint8_t sensorNo, uint8_t sensorState);
  void sendPosition();
  void sendConsigne();
  void sendGo(bool isBlue);
  void sendInit();
  void sendLog(const String& text);
  
  bool process_message(uint8_t data[], uint8_t instruction, uint8_t length);
  void comm_read();

  void registerParameter(float* value, const String& name, void (*setter)(float) = 0);
  void registerParameter(int* value, const String& name, void (*setter)(int) = 0);

  void registerGraph(int graphId, GraphType type, const String& name, String parameterName[], int nbParameters);
  void sendGraphValues(int graphId, float values[], int nbParameters);
  void sendGraphSingleValue(int graphId, int paramId, float value);

private:
  Robot* robot;
  Logger* _logger;

  struct Parameter
  {
      Parameter() : floatValue(0), intValue(0) {}
      Parameter(float* value, const String& paramName, void (*setter)(float) = 0) 
		: floatValue(value), intValue(0), name(paramName), floatSetter() {}
      Parameter(int* value, const String& paramName, void (*setter)(int) = 0) 
		: floatValue(0), intValue(value), name(paramName), intSetter(setter) {}

      float* floatValue;
      int* intValue;
      String name;
	  void (*floatSetter)(float);
	  void (*intSetter)(int);
  };

  Parameter _parameters[MAX_PARAMETERS];
  int _nbRegisteredParameters;
};

#endif

