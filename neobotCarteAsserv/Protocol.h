#ifndef PROTOCOL_H
#define PROTOCOL_H

#define MAX_LENGTH 100
#define SerialCOMM Serial

class Protocol
{
    public:
		Protocol();

		void sendMessage(uint8_t instruction, uint8_t length, uint8_t data[], uint8_t id = 0) const;
		bool read(); // returns true if frame finished
		uint8_t* getData();
		uint8_t getInstruction();
		uint8_t getLength();

		void setCommEnabled(bool value);

    private:
		bool _isEnabled;

		enum MessageStates{FirstFF, SecondFF, Length, Instruction, ReadingData, Checksum};

        uint8_t _currentData[MAX_LENGTH];
        uint8_t _currentInstruction;
        uint8_t _currentLength;
        uint8_t _currentByteRead;
        MessageStates _currentState;
};

#endif // PROTOCOL_H

