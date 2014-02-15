#ifndef COM_H
#define COM_H

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */

#include "Robot.h"

#define MAX_PARAMETERS 10

class Comm
{

public:
  Comm(Robot* r);

  uint8_t* readInt8(uint8_t* data, uint8_t& value);
  uint8_t* readInt16(uint8_t* data, short& value);
  uint8_t* readInt32(uint8_t* data, long& value);
  uint8_t* readFloat(uint8_t* data, float& value);
  uint8_t* writeInt8(uint8_t* data, uint8_t value);
  uint8_t* writeInt16(uint8_t* data, short value);
  uint8_t* writeInt32(uint8_t* data, long value);
  uint8_t* writeFloat(uint8_t* data, float value);
  
  void sendParameters();
  void sendParameterNames();
  void sendAR(uint8_t instruction, bool ok);
  void sendSonars(int ag, int ad, int rg, int rd);
  void sendMicroswitch(bool left, bool right);
  void sendPosition();
  void sendConsigne();
  void sendGo(bool isBlue);
  void restart();
  void quit();
  void sendIsArrived();
  void sendIsBlocked();
  void sendLog(const String& text);
  
  bool process_message(uint8_t data[], uint8_t instruction, uint8_t length);
  void comm_read();

  void registerParameter(float* value, const String& name);
  void registerParameter(int* value, const String& name);

private:
  Robot* robot;

  struct Parameter
  {
      Parameter() : floatValue(0), intValue(0) {}
      Parameter(float* value, const String& paramName) : floatValue(value), intValue(0), name(paramName) {}
      Parameter(int* value, const String& paramName) : floatValue(0), intValue(value), name(paramName) {}

      float* floatValue;
      int* intValue;
      String name;
  };

  Parameter _parameters[MAX_PARAMETERS];
  int _nbRegisteredParameters;
};

#endif
