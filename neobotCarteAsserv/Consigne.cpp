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
	this->_distDemande = 0.0;
    
    _sens = 0.0;
	_dccCoeff = dccCoeff;
	
	this->_phase = Arrive;
	this->_phasePrec = Arrive;
}

float Consigne::calculConsigne(float deltaDistRealise)
{
	this->_consignePrec = this->_consigne;

	// vitesse théorique par rapport à la precedente consigne
	_vitessCourrante = fabs(this->transformeConsigneEnVitesse(_consignePrec));	// mm / ms

	this->majDistAccDcc();
    
    /*
     *
     *  DÈtermination de la phase : accÈlÈration, stationnaire ou freinage
     *
     */

	if (this->calcEstArrive())
	{
		this->_phase = Arrive;
	}
	else if ( // si on est dans la zone de freinage ou si on est trop loin => on freine
				this->doitFreiner == true
			&&
				fabs(this->_distDemande) <= this->_distDcc
		)
	{
		_phase = Transitoire_dcc;
	}
	else if ( // sinon on tente d'accelerer
				this->_vitessCourrante < this->_vitessMaxParcourt
            )
	{
	/*	if (_phase == Transitoire_dcc)
		{
			_phase = Stationaire;
		}
		else
		{*/
			_phase = Transitoire_acc;
		//}
	}
	else
	{
		_phase = Stationaire;
	}

	_phasePrec = _phase;

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
	else if (this->_phase == Transitoire_acc)
	{
		this->_consigne = this->_consignePrec + _sens * this->_variationConsigneMax;
	}
	else if (this->_phase == Transitoire_dcc)
	{
		this->_consigne = this->_consignePrec - _sens * this->_variationConsigneMax;
	}
	else
	{
		this->_consigne = 0.0;
	}

	// limitation consigne max
	if (fabs(this->_consigne) > fabs(this->_consigneMax))
	{
		int sensConsigne = this->_consigne > 0 ? 1 : -1;
		this->_consigne = sensConsigne * this->_consigneMax;
		this->_phase = Stationaire;
	}

	return this->_consigne;
}

void Consigne::setDemande(float dist, bool freinage)
{
	_vitessCourrante = 0.0;
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
    
	if (_phasePrec != Transitoire_dcc)
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
	/****
	 *
	 * Pour test avec modif depuis
	 *
	 */
	this->majConsigneMax();
	this->majVariationConsigneMax();

	float nextVitessCourant = this->_vitessCourrante + this->_accelerationMaxParcourt * this->_periodeMajConsigne; // légère anticipation


	// dist vcc en mm
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

float Consigne::transformeConsigneEnVitesse(float consigne)
{
	return consigne * this->_vitessMax / this->_consigneMax;
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
	return fabs(this->_distDemande) <= this->_distArrive;
}


