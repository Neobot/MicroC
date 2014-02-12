#ifndef SIMULATION_H
#define SIMULATION_H

class Simulation
{
public:
	Simulation(int interval, float mmPerStep, int maxPwm, float maxSpeed, float maxAcceleration);

	int getSteps();
	void setCommande(int pwm);

private:
	float _interval;
	float _mmPerStep;
	float _maxPwm, _neutralPwm;
	float _maxSpeed, _maxAccelPerInterval, _currentSpeed, _pwm;
	int _currentSteps;
};

#endif // SIMULATION_H
