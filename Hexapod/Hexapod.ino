#include "Arduino.h"
#include "DynamixelSerial.h"
#include "QueueList.h"
#include "Parameters.h"
#include "Tools.h"
#include "Leg.h"
#include "Point.h"
#include "Robot.h"
#include "Comm.h"
#include "Task.h"

Task commEcrit(debugInterval);
Task commLect(50);

Robot robot;
Comm comm(&robot);

unsigned long   lTimerStart;        // Start time of the calculation cycles
unsigned long   lTimerEnd;          // End time of the calculation cycles
short           cycleTimeUs;        // Total Cycle time (us)

/*bool 			pause;
int				debugTimer;
int				cyclesCount;*/

void initServos()
{
	Dynamixel.begin(1000000, 2);  // Initialise the servo at 1Mbps and Pin Control 4
}

void setup()
{
	pinMode(13, OUTPUT);		// status led
	digitalWrite(13, HIGH);		// led on
	delay(1000);
	digitalWrite(13, LOW);		// led off

	cycleTimeUs = 10000;
	
	#ifdef ENABLE_SERVOS
	initServos();
	#endif	
	
	while (!robot.initLegs());
	
	digitalWrite(13, HIGH);		// led on
	
	robot.ajoutPoint(0, 500);

	/*pause = true;
	debugTimer = millis();
	cyclesCount = 0;*/
	 
	/*
	// draw reachable area (debug only!)
	leg[4].relPos.z = 0;
	
	for (int yy = -90; yy <= 80; yy++)
		for (int xx = -200; xx <= 200; xx++) {
			leg[4].relPos.x = xx;
			leg[4].relPos.y = yy;
			if (leg[4].inverseKinematics())
				SerialDebug.print("*");
			else
				SerialDebug.print("-");
				
			if (xx == 200)
				SerialDebug.println();
		}
	*/
}

void loop()
{
	lTimerStart = micros();
	
	if (commLect.ready())
		comm.commRead();
	
	if (commEcrit.ready())
		comm.sendPosition();
	
	robot.processGait(cycleTimeUs);
	
	#ifdef ENABLE_DEBUG
	/*if (millis() - debugTimer > debugInterval)
	{
		float avgCycleTime;
		
		if (cyclesCount > 0)
			avgCycleTime = debugInterval*1000/cyclesCount;
		else
			avgCycleTime = 0;
		
		SerialDebug.print("angle error = ");
		SerialDebug.print(angleError*180/PI);
		SerialDebug.print(", pos error = ");
		SerialDebug.println(posError);
		SerialDebug.print("average cycle time = ");
		SerialDebug.print(avgCycleTime);
		SerialDebug.println("us");
		
		debugTimer = millis();
		cyclesCount = 0;
	}*/
		
	/*while(pause || SerialDebug.available())  // If stuff was typed in the serial monitor
  	{
  		if (SerialDebug.available())
  		{
    		bool readChar = (char)SerialDebug.read();
    	
    		//if (readChar == 'p')
    		{
    			if (pause)
    			{
    				SerialDebug.println("Resume received");
    				pause = false;
    			}
    			else
    			{
    				SerialDebug.println("Pause received");
    				pause = true;
    			}
    		}
    	}
 	}*/
	#endif
	
	lTimerEnd = micros();
	if (lTimerEnd > lTimerStart)
		cycleTimeUs = lTimerEnd - lTimerStart;
	else
		cycleTimeUs = 0xffffffffL - lTimerEnd + lTimerStart + 1;

}

