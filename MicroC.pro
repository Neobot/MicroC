#-------------------------------------------------
#
# Project created by QtCreator 2011-09-30T20:51:43
#
#-------------------------------------------------

TEMPLATE = app

SOURCES += \
            neobotCarteAsserv/Consigne.cpp \
            neobotCarteAsserv/neobotCarteAsserv.ino \
            neobotCarteAsserv/Pid.cpp \
            neobotCarteAsserv/Point.cpp \
            neobotCarteAsserv/Protocol.cpp \
			neobotCarteAsserv/Robot.cpp \
            neobotCarteAsserv/Logger.cpp \
            neobotCarteAsserv/Simulation.cpp \
            libraries/AdafruitTCS34725master/Adafruit_TCS34725.cpp \
			libraries/PWM01/pwm01.cpp \
			libraries/Task/Task.cpp \
			TestColorSensor/TestColorSensor.ino \
			TestMoteursDue/TestMoteursDue.ino \
			neobotCarteAsserv/Comm.cpp


SOURCES += \
			neobotCarteAsserv/Comm.h \
			neobotCarteAsserv/Consigne.h \
			neobotCarteAsserv/Pid.h \
            neobotCarteAsserv/Point.h \
            neobotCarteAsserv/Protocol.h \
            neobotCarteAsserv/Robot.h \
            neobotCarteAsserv/Logger.h \
            neobotCarteAsserv/Simulation.h \
			neobotCarteAsserv/IOConfig.h \
			neobotCarteAsserv/Parameters.h \
			neobotCarteAsserv/Instructions.h \
			libraries/AdafruitTCS34725master/Adafruit_TCS34725.h \
			libraries/QueueList/QueueList.h \
			libraries/PWM01/pwm01.h \
			libraries/Task/Task.h
