#include "Arduino.h"
#include "Task.h"

Task::Task(unsigned long interval_ms)
{
	_interval = interval_ms;
}

char Task::ready()
{
	if (millis() - _previous_millis >= _interval)
	{
    	_previous_millis = millis();
    	return 1;
    }
    
	return 0;
}