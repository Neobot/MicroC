#include "Arduino.h"
#include "Consigne.h"

#define CONSIGNE_FREINAGE 0.0

Consigne::Consigne(float vmax, float amax, float periodeConsigne, float dccCoeff, float distArrive)
{
	this->_vitessMax = 1.4; //Vitesse maximum que peut atteindre le robot
	this->_vitessCourrante = 0.0;
	this->_periodeMajConsigne = periodeConsigne;
	this->_accelerationMaxParcourt = amax;
	this->_distArrive = distArrive;
	this->doitFreiner = true;
	
	this->setVmaxParcourt(vmax);
	
	this->_consigne = 0.0;
	this->_consignePrec = 0.0;
	this->_distRealise = 0.0;
	this->_distDemande = 0.0;
	_dccCoeff = dccCoeff;
	
	this->_phase = Arrive;
}

float Consigne::calculConsigne(float deltaDistRealise)
{
	this->_consignePrec = this->_consigne;
	this->_distRealise += deltaDistRealise;

	//float newVitesseCourante = _vitessCourrante + fabs(deltaDistRealise) / this->_periodeMajConsigne;
	//newVitesseCourante /= 2;

	float newVitesseCourante = fabs(deltaDistRealise) / this->_periodeMajConsigne;

	if (newVitesseCourante <= _vitessMax * 2)
		_vitessCourrante = newVitesseCourante;
	else
	{
		SerialUSB.print("ERREUR en vitesse: ");
		SerialUSB.print(newVitesseCourante);
		SerialUSB.print(" found at ");
		SerialUSB.print(_distRealise);
		SerialUSB.println("mm");
	}

	this->majDistAccDcc();
	this->majVariationConsigneMax(fabs(this->_distDemande - this->_distRealise));

	int sens = _distDemande - _distRealise > 0 ? 1 : -1;
	Phase phasePrec = _phase;
	bool accelPrec = _acceleration;
	_acceleration = true;


	//calcul de la phase
	if (this->calcEstArrive())
	{
		this->_phase = Arrive;
	}

	// si on est dans la zone de freinage ou si on est trop loin
	else if (this->doitFreiner == true && (
				 fabs(this->_distDemande - this->_distRealise) <= (this->_distDcc) ||
				 fabs(this->_distDemande) < fabs(this->_distRealise)  &&
				 fabs(this->_distDemande) * this->_distRealise == fabs(this->_distRealise) * this->_distDemande))
	{
		this->_phase = Transitoire;
		_acceleration = false;
	}

	else if (fabs(_distRealise) <= _distAcc || this->_vitessCourrante < this->_vitessMaxParcourt)
	{
		if (phasePrec == Transitoire && !accelPrec)
		{
			_phase = Stationaire;
			this->_consignePrec = this->transformeDeltaDistanceEnConsigne(deltaDistRealise);
		}
		else
		{
			this->_phase = Transitoire;
			_acceleration = true;
		}
	}

	if (this->_phase == Arrive)
	{
		this->_consigne = 0.0;
	}
	else if (this->_phase == Stationaire)
	{
		this->_consigne = this->_consignePrec;
	}
	else if (this->_phase == Transitoire)
	{

		// acceleration ou freinage
		if (_acceleration)
		{
			this->_consigne = this->_consignePrec + this->_variationConsigneMax;
		}
		else
		{
			this->_consigne = (float)CONSIGNE_FREINAGE * sens;
		}

		// limitation consigne max
		if (fabs(this->_consigne) > fabs(this->_consigneMax))
		{
			int sensConsigne = this->_consigne > 0 ? 1 : -1;
			this->_consigne = sensConsigne * this->_consigneMax;
			this->_phase = Stationaire;
		}

	}
	else
	{
		this->_consigne = 0.0;
	}

	return this->_consigne;
}

void Consigne::setDemande(float dist, bool freinage)
{
	_vitessCourrante = 0.0;
	this->_distRealise = 0.0;
	this->_distDemande = dist;
	this->_phase = Stationaire;
	this->doitFreiner = freinage;

	/*if (this->doitFreiner)
  {
	this->_distArrive = DIST_ARRIVE_AVEC_FREINAGE;
  }
  else
  {
	this->_distArrive = DIST_ARRIVE_SANS_FREINAGE;
  }*/

	this->majVariationConsigneMax(dist);
	
	if (dist > 0.0)
	{
		this->_variationConsigneMax = fabs(this->_variationConsigneMax);
	}
	else if (dist < 0.0)
	{
		this->_variationConsigneMax = -1.0 * fabs(this->_variationConsigneMax);
	}
	else
	{
		this->_variationConsigneMax = 0;
	}

}

void Consigne::setVmaxParcourt(float vmax)
{
	this->_vitessMaxParcourt = vmax;
	this->majConsigneMax();
	this->majVariationConsigneMax();
}

void Consigne::setAmaxParcourt(float amax)
{
	this->_accelerationMaxParcourt = amax;
	this->majVariationConsigneMax();
}


void Consigne::majDistAccDcc()
{
	this->_distAcc = this->_vitessMaxParcourt * this->_vitessMaxParcourt / (2.0 * this->_accelerationMaxParcourt);

	float nextVitessCourant = this->_vitessCourrante + this->_accelerationMaxParcourt * this->_periodeMajConsigne;

	if (nextVitessCourant > this->_vitessMaxParcourt)
	{
		nextVitessCourant = this->_vitessMaxParcourt;
	}

	this->_distDcc = _dccCoeff * (_vitessCourrante * _vitessCourrante) / (2.0 * this->_accelerationMaxParcourt) + 1.1 * _vitessCourrante * this->_periodeMajConsigne;// + DIST_ARRIVE_AVEC_FREINAGE;
}

void Consigne::majConsigneMax()
{
	this->_consigneMax = CONSIGNE_MAX * this->_vitessMaxParcourt / this->_vitessMax;
}

void Consigne::majVariationConsigneMax(float distRestante)
{
	float accMax = distRestante / (this->_periodeMajConsigne * this->_periodeMajConsigne);
	float sens = fabs(this->_variationConsigneMax) > 0 ? fabs(this->_variationConsigneMax) / this->_variationConsigneMax : 1;

	if (accMax >  this->_accelerationMaxParcourt)
	{
		accMax = this->_accelerationMaxParcourt;
	}

	this->_variationConsigneMax = this->_consigneMax * accMax * this->_periodeMajConsigne / this->_vitessMaxParcourt;

	if (this->_variationConsigneMax > _consigneMax)
	{
		this->_variationConsigneMax = _consigneMax;
	}

	this->_variationConsigneMax = sens * this->_variationConsigneMax;
}

float Consigne::transformeDeltaDistanceEnConsigne(float delta)
{
	return delta * this->_consigneMax / (this->_vitessMax * this->_periodeMajConsigne);
}

bool Consigne::estArrive()
{
	if (this->_phase == Arrive)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Consigne::calcEstArrive()
{
	return fabs(this->_distDemande - this->_distRealise) <= this->_distArrive;
}

