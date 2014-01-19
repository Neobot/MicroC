#ifndef Task_h
#define Task_h

class Task
{

public:
	Task(unsigned long interval);
	char ready();
	
private:
	unsigned long  _previous_millis, _interval;

};

#endif
