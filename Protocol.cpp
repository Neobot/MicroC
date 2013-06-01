#include "Arduino.h"
#include "Protocol.h"

Protocol::Protocol()
{
	_currentState = FirstFF;
}


void Protocol::sendMessage(uint8_t instruction, uint8_t length, uint8_t data[], uint8_t id) const
{
    uint8_t frameData[20];
    int i;
    uint8_t checksum;

    frameData[0] = (uint8_t)0xFF;
    frameData[1] = (uint8_t)0xFF;
    frameData[2] = length + 1;
    frameData[3] = instruction;
    
    for (i = 0; i < length; i++)
    	frameData[i+4] = data[i];
    
    // calculate the checksum
    int tempChecksum = 0;
    for(i = 0; i < length; i++)
        tempChecksum += data[i];
    tempChecksum += length + 1;
    tempChecksum += instruction;
    checksum = 255 - (uint8_t)(tempChecksum % 256);

    frameData[length + 4] = checksum;

    // Write the data to the serial port
    SerialUSB.write(frameData, length+5);
}


uint8_t Protocol::read()
{
    while (SerialUSB.available() > 0)
    {
        uint8_t value;
        
        value = SerialUSB.read();

        switch(_currentState)
        {
        case FirstFF:
        case SecondFF:          
            if (value != 0xFF)
            {
                _currentState = FirstFF;
                break;
            }
            _currentByteRead = 0;
            _currentInstruction = 0;
            if (_currentState == FirstFF)
                _currentState = SecondFF;
            else
                _currentState = Length;
            continue;
            
        case Length:
            if (value > MAX_LENGTH)
            {
                _currentState = FirstFF;
                continue;
            }
             _currentLength = value;
             _currentData[0] = value;
             _currentState = Instruction;
             continue;
             
        case Instruction:
             _currentInstruction = value;
             _currentData[1] = _currentInstruction;
             if (_currentLength > 1)
                _currentState = ReadingData;
             else
                 _currentState = Checksum;
             continue;
             
         case ReadingData:
             if (_currentLength == 0)
             {
                 _currentState = Checksum;
                 continue;
             }

             _currentData[_currentByteRead] = value;
             ++_currentByteRead;
             
             if (_currentByteRead == _currentLength - 1)
                 _currentState = Checksum;
             continue;
             
         case Checksum:
         // calculate the checksum
    		int tempChecksum;
    		uint8_t calcChecksum;
    		for(int i = 0; i < _currentLength - 1; i++)
        		tempChecksum += _currentData[i];
		tempChecksum += _currentLength;
		tempChecksum += _currentInstruction;
    		calcChecksum = 255 - (uint8_t)(tempChecksum % 256);
    		
             _currentState = FirstFF;
             //if (value != calcChecksum)
             //    continue;
                 
             return 1;

             //continue;
         }
    }
    
    return 0;
}

uint8_t * Protocol::getData()
{
	return _currentData;
}

uint8_t Protocol::getInstruction()
{
	return _currentInstruction;
}

uint8_t Protocol::getLength()
{
	return _currentLength - 1;
}
