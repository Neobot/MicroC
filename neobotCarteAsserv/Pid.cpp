/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */
 
/*
 * Project : Neobot
 * Version : 0.42
 * Date : 30/12/2012
 * Author : Neobot
 */

#include "Arduino.h"
#include "Pid.h"

PID::PID(bool actif, float kp, float kd, float ki)
{
	this->changeEtat(actif);
	
	this->_kp = kp;
	this->_kd = kd;
    this->_ki = ki;
    _valMaxCorrection = VALUE_MAX_PID;
}

float PID::calculCommande(float consigne, float distanceRealiseEnNormeConsigne)
{
	_correction = 0.0;
	if (_etatCourant == Actif)
	{
          float derive;
          float integral;
          
		  _lastErreur = _erreur;
		  _erreur = _precedenteConsigne - distanceRealiseEnNormeConsigne;
          
		  derive = _erreur - _lastErreur; // a voir si pas mieux this->_erreur - this->_lastErreur

          integral = 0.0;
          for(int i = 0; i < NB_VALUE_FOR_PID_INTEGRAL; ++i)
			integral += _prevErreurs[i];
          
		  _correction = _kp * _erreur + _kd * derive + _ki * integral;
          
		  //seuilPid();
          
		  addPrevErreur();
	}
		_precedentePrecedenteConsigne = _precedenteConsigne;
		_precedenteConsigne = consigne;
        
        _commande = consigne + _correction;
        
	return _commande;
}

void PID::seuilPid()
{
    if ( this->_correction >= 0)
    {
        this->_correction = min(this->_correction, this->_valMaxCorrection);
    }
    else
    {
        this->_correction = max(this->_correction, -1.0 * this->_valMaxCorrection);
    }
}

void PID::reset()
{
	this->_erreur = 0.0;
	this->_correction = 0.0;
    this->_lastErreur = 0.0;
    this->_index = 0;
    
    _precedenteConsigne = 0.0;
	_precedentePrecedenteConsigne = 0.0;
    
    for(int i = 0; i < NB_VALUE_FOR_PID_INTEGRAL; ++i)
        this->_prevErreurs[i] = 0;
}

void PID::addPrevErreur()
{
    ++this->_index;
    if (this->_index >= NB_VALUE_FOR_PID_INTEGRAL)
      this->_index = 0;
      
    this->_prevErreurs[this->_index] = this->_erreur;
}

void PID::changeEtat(bool actif)
{
    this->reset();
	if (actif)
		this->_etatCourant = Actif;
	else
		this->_etatCourant = Desactive;
}


