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
}

float PID::calculCommande(float consigne, float distanceRealiseEnNormeConsigne)
{
	this->_commande = consigne;
	
	if (this->_etatCourant == Actif)
	{
        float derive;
        float integral;
        
        this->_lastErreur = this->_erreur;   
		this->_erreur = this->_precedenteConsigne - distanceRealiseEnNormeConsigne;
        
        derive = this->_erreur - this->_lastErreur;
        
        integral = 0.0;
        for(int i = 0; i < NB_VALUE_FOR_PID_INTEGRAL; ++i)
          integral += this->_prevErreurs[i];
        
        this->_correction = his->_kp * this->_erreur + this->_kd * derive + this->_ki * integral;  
        
        this->seuilPid();
        
        this->addPrevErreur();
	}
	
	return consigne + this->_correction;
}

float PID::seuilPid()
{
    if ( this->_correction > 0)
    {
        this->_correction = min(this->_correction, this->_valMaxCorrection);
    }
    else
    {
        this->_correction = max(this->_correction, this->_valMaxCorrection);
    }
}

void PID::reset()
{
	this->_erreur = 0.0;
	this->_correction = 0.0;
    this->_lastErreur = 0.0;
    this->_index = 0;
    
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

