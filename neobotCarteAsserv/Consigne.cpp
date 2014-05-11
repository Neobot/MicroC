#include "Arduino.h"
#include "Consigne.h"

Consigne::Consigne(float vmax, float amax, float periodeConsigne, float dccCoeff, float distArrive)
{
	this->_vitessMax = VITESSE_MAX_REEL; //Vitesse maximum que peut atteindre le robot
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
    
    _sens = 0.0;
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

	if (newVitesseCourante <= _vitessMax)
		_vitessCourrante = newVitesseCourante;
	else
	{
        _vitessCourrante = _vitessMax;
	}

	this->majDistAccDcc();


	bool accelPrec = _acceleration;
	_acceleration = true;

    
    /*
     *
     *  Détermination de la phase : accélération, stationnaire ou freinage
     *
     */

	if (this->calcEstArrive())
	{
		this->_phase = Arrive;
	}
	else if ( // si on est dans la zone de freinage ou si on est trop loin => on freine
                    this->doitFreiner == true 
                && 
                (
                        fabs(this->_distDemande - this->_distRealise) <= this->_distDcc
                    ||
                            fabs(this->_distDemande) < fabs(this->_distRealise)  
                        &&
                            fabs(this->_distDemande) * this->_distRealise == fabs(this->_distRealise) * this->_distDemande
                )
            )
	{
		_phase = Transitoire;
		_acceleration = false;
	}
	else if ( // sinon on tente d'accelerer
                fabs(_distRealise) <= _distAcc || this->_vitessCourrante < this->_vitessMaxParcourt
            )
	{
		if (_phase == Transitoire && !accelPrec)
		{
			_phase = Stationaire;
		}
		else
		{
			_phase = Transitoire;
		}
	}


    /*
     *
     *  Calcule de la consigne en fonction de la phase de deplacement
     *
     */
    
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
			this->_consigne = this->_consignePrec + _sens * this->_variationConsigneMax;
		}
		else
		{
			this->_consigne = this->_consignePrec - _sens * this->_variationConsigneMax;
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

	if (this->doitFreiner)
    {
        this->_distArrive = DIST_ARRIVE_AVEC_FREINAGE;
    }
    else
    {
        this->_distArrive = DIST_ARRIVE_SANS_FREINAGE;
    }
    
    _sens = dist != 0.0 ? fabs(dist) / dist : 0.0;
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

	this->_distDcc = _dccCoeff * (nextVitessCourant * nextVitessCourant) / (2.0 * this->_accelerationMaxParcourt);// + DIST_ARRIVE_AVEC_FREINAGE;
}

void Consigne::majConsigneMax()
{
	this->_consigneMax = this->_vitessMaxParcourt * COMMANDE_MOTEUR_MAX / this->_vitessMax;
}

void Consigne::majVariationConsigneMax()
{
	this->_variationConsigneMax = this->_accelerationMaxParcourt * COMMANDE_MOTEUR_MAX / this->_vitessMax;

	if (this->_variationConsigneMax > _consigneMax)
	{
		this->_variationConsigneMax = _consigneMax;
	}
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

